//! Deterministic top-down car simulation, compiled twice from one source:
//!
//! * as an `rlib` linked into the **sidecar**, which is the authority;
//! * as a `cdylib` for `wasm32-unknown-unknown`, loaded by the **browser**,
//!   which predicts the local car and rolls back when the authority disagrees.
//!
//! Because both sides run the same instructions on the same inputs, prediction
//! error on a healthy connection is exactly zero, and the reconciliation code
//! only has to earn its keep when packets are late, dropped or tampered with.

pub mod aero;
pub mod bot;
pub mod car;
pub mod collide;
pub mod damage;
pub mod drivetrain;
pub mod ffi;
pub mod fingerprint;
pub mod math;
pub mod suspension;
pub mod tire;
pub mod track;
pub mod wheel;
pub mod world;

pub use car::{CarInput, CarState, CAR_FLOATS, DT, INPUT_FLOATS, TICK_HZ};
pub use fingerprint::fingerprint;
pub use math::V2;
pub use track::{Track, CHECKPOINTS, SAMPLES};
pub use world::{World, MAX_CARS};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bot::BotBrain;
    use crate::damage;
    use crate::drivetrain;
    use crate::math::V2;

    fn scripted_input(tick: u64) -> CarInput {
        // A lap-ish sequence that exercises power, braking, and a handbrake flick.
        let t = tick as f32 * DT;
        CarInput {
            throttle: if (t % 7.0) < 5.0 { 1.0 } else { 0.0 },
            steer: math::sin(t * 0.9) * 0.8,
            brake: if (t % 7.0) >= 5.0 { 0.6 } else { 0.0 },
            handbrake: if (t % 11.0) > 10.4 { 1.0 } else { 0.0 },
        }
    }

    /// One steady-state cornering run: hold `v` and `steer` until it settles,
    /// then report lateral g, both slip angles and how far the body is leaning.
    fn corner_at(v: f32, steer: f32) -> (f32, f32, f32, f32) {
        let mut c = CarState::default();
        c.vx = v;
        c.sync_drivetrain();
        for _ in 0..1500 {
            let err = v - c.speed();
            let inp = CarInput {
                throttle: math::clamp(err * 0.3, 0.0, 1.0),
                steer,
                brake: math::clamp(-err * 0.1, 0.0, 1.0),
                handbrake: 0.0,
            };
            for _ in 0..car::SUBSTEPS {
                car::integrate(&mut c, &inp, car::H);
            }
        }
        (math::abs(c.ay) / car::G, c.slip_f, c.slip_r, c.roll)
    }

    /// Sweep the lock at a fixed speed and keep the best run. Returns the lock
    /// it happened at, the peak in g, both slip angles and the roll angle.
    fn skidpad(v: f32) -> (f32, f32, f32, f32, f32) {
        let mut best = (0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.0f32);
        let mut s = 0.1f32;
        while s <= 1.001 {
            let (g, sf, sr, roll) = corner_at(v, s);
            if g > best.1 {
                best = (s, g, sf, sr, roll);
            }
            s += 0.1;
        }
        best
    }

    #[test]
    fn track_geometry_is_sane() {
        let t = Track::new();
        assert!(
            t.length > 1200.0 && t.length < 1700.0,
            "lap length {}",
            t.length
        );
        for i in 0..SAMPLES {
            assert!(t.half_width[i] > 6.0 && t.half_width[i] < 11.0);
            assert!(t.tangent[i].len() > 0.99);
        }
        // Uniform arc-length sampling is what the rest of the code assumes.
        for i in 1..SAMPLES {
            let d = t.p[i].sub(t.p[i - 1]).len();
            assert!((d - t.ds).abs() < t.ds * 0.05, "sample {i} spacing {d}");
        }
        println!(
            "lap {:.1} m, ds {:.2} m, width {:.1}..{:.1} m",
            t.length,
            t.ds,
            t.half_width.iter().cloned().fold(f32::MAX, f32::min) * 2.0,
            t.half_width.iter().cloned().fold(0.0, f32::max) * 2.0
        );
    }

    #[test]
    fn nearest_with_hint_matches_full_search() {
        let t = Track::new();
        for i in (0..SAMPLES).step_by(7) {
            let p = t.p[i].add(t.normal[i].scale(3.0));
            let full = t.nearest(p, track::NO_HINT);
            let hinted = t.nearest(p, i as u16);
            assert_eq!(full.idx, hinted.idx);
            assert!((full.s - hinted.s).abs() < 1e-3);
        }
    }

    #[test]
    fn car_accelerates_and_tops_out() {
        // Free space: engine, gearbox, tires and air, but no track and no
        // walls. Every number below is an emergent one -- nothing in the model
        // is told what the car's performance ought to be.
        let mut c = CarState::default();
        let inp = CarInput {
            throttle: 1.0,
            ..Default::default()
        };
        let (mut to_100, mut to_200) = (0.0f32, 0.0f32);
        let mut shifts = 0;
        let mut gear = c.gear;
        for i in 0..60 * 90 {
            for _ in 0..car::SUBSTEPS {
                car::integrate(&mut c, &inp, car::H);
            }
            let kmh = c.forward_speed() * 3.6;
            if to_100 == 0.0 && kmh >= 100.0 {
                to_100 = i as f32 * DT;
            }
            if to_200 == 0.0 && kmh >= 200.0 {
                to_200 = i as f32 * DT;
            }
            if c.gear != gear {
                shifts += 1;
                gear = c.gear;
            }
        }
        let top = c.forward_speed();
        println!(
            "0-100 km/h {to_100:.2} s, 0-200 {to_200:.2} s, top {:.0} km/h in gear {} at {:.0} rpm ({shifts} shifts)",
            top * 3.6,
            c.gear as i32,
            c.engine / drivetrain::RPM_TO_RAD
        );
        // A traction-limited launch on a 350 hp rear-drive car. Much faster
        // than this would mean the tires are not the limit they should be.
        assert!(to_100 > 3.0 && to_100 < 6.5, "0-100 km/h in {to_100:.2} s");
        assert!(to_200 > 9.0 && to_200 < 24.0, "0-200 km/h in {to_200:.2} s");
        // Top speed is where drive force meets drag, not where the gearing runs
        // out -- the limiter in sixth is deliberately beyond it.
        assert!(top > 68.0 && top < 88.0, "top speed {top:.1} m/s");
        assert!(shifts >= 5, "only {shifts} gear changes reaching top speed");
    }

    #[test]
    fn a_provoked_spin_scrubs_off_and_stops() {
        // Full lock and the handbrake at 144 km/h is the worst a player can do
        // to themselves, and the car should absolutely spin. What it must not
        // do is keep spinning: the tires scrub, the speed goes, the rotation
        // stops, and the driver is left somewhere with far less energy than
        // they started with.
        let mut c = CarState::default();
        c.vx = 40.0;
        c.sync_drivetrain();
        let mut yaw = 0.0f32;
        let mut prev = c.heading;
        for tick in 0..600u64 {
            let held = tick < 90;
            let inp = CarInput {
                throttle: 0.0,
                steer: if held { 1.0 } else { 0.0 },
                brake: 0.0,
                handbrake: if held { 1.0 } else { 0.0 },
            };
            for _ in 0..car::SUBSTEPS {
                car::integrate(&mut c, &inp, car::H);
            }
            yaw += math::wrap_pi(c.heading - prev);
            prev = c.heading;
        }
        let turns = yaw.abs().to_degrees() / 360.0;
        println!(
            "spin: {:.0} deg ({turns:.2} turns), 40.0 -> {:.1} m/s, omega {:.3}",
            yaw.to_degrees(),
            c.speed(),
            c.omega
        );
        // The guarantee worth having is about energy, not revolutions. A real
        // car given full lock and the handbrake at 144 km/h spins, and the old
        // model only kept it under three quarters of a turn because it had an
        // explicit damping term standing in for tire scrub. The Magic Formula
        // supplies that scrub for real, so the spin is allowed to be a spin --
        // what it may not do is leave the driver pointing the wrong way with
        // the speed still on.
        assert!(turns < 2.0, "car spun {turns:.2} turns");
        // It comes out of the spin rolling *backwards* in first, at a speed
        // the drivetrain can do nothing about: a forward gear driven the wrong
        // way would stall the engine, and the clutch lets go rather than let
        // it. So the remaining crawl is coasting on rolling resistance, and
        // the bound is on energy -- under 8 m/s is 4% of what it arrived with.
        assert!(c.speed() < 8.0, "still doing {:.1} m/s after a spin", c.speed());
        assert!(c.omega.abs() < 0.05, "still rotating at {:.3} rad/s", c.omega);
    }

    #[test]
    fn peak_grip_is_real_and_the_car_understeers_past_it() {
        // Steady-state skidpad, sweeping lock to find where the limit actually
        // is. Sweeping matters now: with a real tire curve, more steering past
        // the peak buys *less* lateral force, so a single fixed input would be
        // measuring understeer rather than grip.
        let (at, peak, slip_f, slip_r, roll) = skidpad(32.0);
        let beyond = corner_at(32.0, 1.0).0;
        println!(
            "skidpad at 32 m/s: {peak:.2} g at {at:.1} lock (slip f {:.1} deg r {:.1} deg, roll {:.1} deg); {beyond:.2} g at full lock",
            slip_f.to_degrees(),
            slip_r.to_degrees(),
            roll.to_degrees()
        );
        assert!(peak > 1.30, "only {peak:.2} g available");
        // Past the peak the front gives up first. An oversteer-limited car is
        // undriveable with a keyboard, and avoiding it is what the roll
        // stiffness split in `suspension.rs` is set up for.
        assert!(
            slip_f.abs() > slip_r.abs(),
            "car oversteers at the limit (front {slip_f:.3} rear {slip_r:.3})"
        );
        assert!(beyond < peak, "lateral g still climbing at full lock");
    }

    /// Downforce has to show up as cornering speed, not just as a bigger number
    /// in `aero.rs`.
    #[test]
    fn a_fast_corner_holds_more_g_than_a_slow_one() {
        let slow = skidpad(20.0).1;
        let fast = skidpad(60.0).1;
        println!("peak lateral: {slow:.2} g at 20 m/s, {fast:.2} g at 60 m/s");
        assert!(fast > slow * 1.05, "downforce bought no cornering speed");
    }

    /// A car settled in a steady corner at `v` and `steer`, throttle modulated
    /// to hold the speed, for handing to a driver.
    fn settled(v: f32, steer: f32) -> CarState {
        let mut c = CarState::default();
        c.vx = v;
        c.sync_drivetrain();
        for _ in 0..600 {
            let err = v - c.speed();
            let inp = CarInput {
                throttle: math::clamp(err * 0.3, 0.0, 1.0),
                steer,
                brake: math::clamp(-err * 0.1, 0.0, 1.0),
                handbrake: 0.0,
            };
            for _ in 0..car::SUBSTEPS {
                car::integrate(&mut c, &inp, car::H);
            }
        }
        c
    }

    /// Three seconds of a keyboard driver: a throttle that is a switch, held
    /// at `throttle`; the steering key held at `steer` until the rear steps out
    /// past ten degrees, and from then on opposite lock while it is more than
    /// eight degrees out and nothing below three -- every decision reaching
    /// the car a quarter of a second after it was taken. Returns the worst
    /// body slip angle reached, in degrees, and whether the car was travelling
    /// within five degrees of where it pointed, rear included, for the last
    /// half second.
    fn keyboard_driver(mut c: CarState, steer: f32, throttle: f32, handbrake_ticks: u32) -> (f32, bool) {
        const REACT: usize = 15;
        let mut queue = std::collections::VecDeque::new();
        let mut noticed = false;
        let mut hold = steer;
        let mut worst = 0.0f32;
        let mut calm = 0;
        for i in 0..180u32 {
            let sr = c.slip_r;
            noticed |= math::abs(sr) > 0.17;
            let want = if !noticed {
                steer
            } else if math::abs(sr) > 0.14 {
                -math::signum(sr)
            } else if math::abs(sr) < 0.05 {
                0.0
            } else {
                hold
            };
            hold = want;
            queue.push_back(want);
            let s = if queue.len() > REACT { queue.pop_front().unwrap() } else { steer };
            let inp = CarInput {
                throttle,
                steer: s,
                brake: 0.0,
                handbrake: if i < handbrake_ticks { 1.0 } else { 0.0 },
            };
            for _ in 0..car::SUBSTEPS {
                car::integrate(&mut c, &inp, car::H);
            }
            let vb = c.vel().to_local(c.heading);
            let beta = math::abs(math::atan2(vb.y, vb.x));
            worst = math::max(worst, beta);
            calm = if beta < 0.09 && math::abs(c.slip_r) < 0.09 { calm + 1 } else { 0 };
        }
        (worst.to_degrees(), calm >= 30)
    }

    /// The report that started the audit: "sliding out way too much after
    /// turns as soon as I touch the accelerator". A keyboard throttle is a
    /// switch, so this is a switch: mid-corner, the pedal goes from holding
    /// speed to flat and stays there, with the steering key held where it
    /// was. Traction control has to keep the rear on the road. It used to
    /// start closing at nearly twice the tire's peak slip, and the inside rear
    /// ran at a quarter slip until the outside one let go too.
    #[test]
    fn a_keyboard_throttle_mid_corner_does_not_spin_the_car() {
        for v in [10.0f32, 15.0, 22.0] {
            let c = settled(v, 0.65);
            let (worst, _) = keyboard_driver(c, 0.65, 1.0, 0);
            println!("{v:.0} m/s, throttle switched on mid-corner: worst body slip {worst:.0} deg");
            assert!(worst < 10.0, "at {v} m/s the rear stepped out to {worst:.0} deg of body slip");
        }
    }

    /// The second half of it: "once I start to slide out, no amount of
    /// reactive steering changes anything". Flick the handbrake mid-corner for
    /// two hundred milliseconds and hand the slide to the same driver. Before
    /// `car::steer_aid` measured opposite lock from where the fronts were going
    /// and damped the slide, this left the car forty degrees sideways at
    /// 20 m/s with full opposite lock on, and spinning at 30.
    #[test]
    fn a_keyboard_driver_can_catch_a_slide() {
        for v in [20.0f32, 30.0] {
            let c = settled(v, 0.6);
            let (worst, calm) = keyboard_driver(c, 0.6, 0.3, 12);
            println!("{v:.0} m/s, handbrake flicked mid-corner: worst body slip {worst:.0} deg, caught: {calm}");
            assert!(worst < 30.0, "at {v} m/s the car went {worst:.0} deg sideways");
            assert!(calm, "at {v} m/s the car was still sliding three seconds later");
        }
    }

    /// The launch is the same car at any substep. The engine and the rear
    /// wheels used to be stepped one body at a time, each charged for a
    /// change in clutch torque that the other one's motion cancelled, and a
    /// fifth of a g went missing through first gear at 480 Hz that came back
    /// at 2 kHz -- see `drivetrain.rs`.
    #[test]
    fn the_launch_does_not_depend_on_the_substep() {
        let to_100 = |div: u32| {
            let mut c = CarState::default();
            let inp = CarInput { throttle: 1.0, ..Default::default() };
            let h = car::H / div as f32;
            for i in 0..60 * 8 {
                for _ in 0..car::SUBSTEPS * div {
                    car::integrate(&mut c, &inp, h);
                }
                if c.forward_speed() * 3.6 >= 100.0 {
                    return (i + 1) as f32 * DT;
                }
            }
            f32::MAX
        };
        let coarse = to_100(1);
        let fine = to_100(4);
        println!("0-100 km/h: {coarse:.2} s at 480 Hz, {fine:.2} s at 1920 Hz");
        assert!(math::abs(coarse - fine) < 0.1, "the launch depends on the substep");
    }

    /// Braking distance from a real tire curve, with ABS keeping the wheels on
    /// the useful side of the peak.
    #[test]
    fn it_stops_from_speed_in_a_realistic_distance() {
        let mut c = CarState::default();
        c.vx = 50.0;
        c.sync_drivetrain();
        let mut dist = 0.0f32;
        let mut ticks = 0;
        for _ in 0..900 {
            let inp = CarInput {
                brake: 1.0,
                ..Default::default()
            };
            let before = c.x;
            for _ in 0..car::SUBSTEPS {
                car::integrate(&mut c, &inp, car::H);
            }
            dist += c.x - before;
            ticks += 1;
            if c.forward_speed() < 0.5 {
                break;
            }
        }
        let g = 50.0 / (ticks as f32 * DT) / car::G;
        println!(
            "50 m/s to a stop: {dist:.1} m in {:.2} s ({g:.2} g average), front wheel still turning at {:.1} rad/s",
            ticks as f32 * DT,
            c.w_fl
        );
        assert!(dist > 50.0 && dist < 115.0, "stopped in {dist:.1} m");
        assert!(g > 1.0, "only {g:.2} g of braking");
    }

    #[test]
    fn rollback_replay_is_bit_exact() {
        // This is the invariant the whole netcode rests on: re-simulating from
        // an older state with the same inputs must reproduce the same bits.
        let mut a = World::new();
        a.spawn(0, 0);
        let mut states = Vec::new();
        let mut inputs = Vec::new();
        for tick in 0..400u64 {
            let inp = scripted_input(tick);
            inputs.push(inp);
            a.inputs[0] = inp;
            a.step(1);
            states.push(a.cars[0]);
        }

        // Rewind to tick 150 and replay the recorded inputs.
        let mut b = World::new();
        b.spawn(0, 0);
        b.cars[0] = states[149];
        b.tick = 150;
        for tick in 150..400usize {
            b.inputs[0] = inputs[tick];
            b.step(1);
        }
        assert_eq!(
            b.cars[0], states[399],
            "replayed state diverged from the original"
        );
        assert_eq!(
            world::checksum(&[b.cars[0]]),
            world::checksum(&[states[399]])
        );
    }

    #[test]
    fn simulation_is_repeatable() {
        let run = || {
            let mut w = World::new();
            w.spawn(0, 0);
            w.spawn(1, 1);
            let brain = BotBrain::new(7);
            for tick in 0..900u64 {
                w.inputs[0] = scripted_input(tick);
                w.inputs[1] = brain.drive(1, &w.cars, w.active, &w.track, tick);
                w.step(0b11);
            }
            world::checksum(&w.cars[0..2])
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn bots_race_each_other_around_the_circuit() {
        let mut w = World::new();
        let n = 8usize;
        let brains: Vec<BotBrain> = (0..n as u32).map(BotBrain::new).collect();
        for i in 0..n {
            w.spawn(i, i);
        }
        let mut worst_overshoot = 0.0f32;
        let mut stuck_ticks = 0;
        for tick in 0..60 * 150u64 {
            for i in 0..n {
                w.inputs[i] = brains[i].drive(i, &w.cars, w.active, &w.track, tick);
            }
            w.step((1 << n) - 1);
            for i in 0..n {
                let hit = w.track.nearest(w.cars[i].pos(), w.cars[i].seg as u16);
                worst_overshoot = worst_overshoot.max(w.cars[i].lat.abs() - hit.half_width);
                if tick > 180 && w.cars[i].speed() < 5.0 {
                    stuck_ticks += 1;
                }
            }
        }
        let best: Vec<f32> = (0..n).map(|i| w.cars[i].best_lap).collect();
        let laps: Vec<f32> = (0..n).map(|i| w.cars[i].lap).collect();
        println!("laps {laps:?}
best {best:?}
stuck ticks {stuck_ticks}, worst wall overshoot {worst_overshoot:.2} m");
        assert!(
            worst_overshoot < 1.2,
            "a bot ended up {worst_overshoot:.2} m outside the barrier"
        );
        for (i, b) in best.iter().enumerate() {
            assert!(
                *b > 30.0 && *b < 75.0,
                "bot {i} lapped in {b:.1}s, which is not racing"
            );
        }
        let total = 60 * 150 * n as i32;
        assert!(
            stuck_ticks * 100 < total,
            "bots spent {}% of the race crawling",
            stuck_ticks * 100 / total
        );
    }

    #[test]
    fn walls_keep_cars_inside() {
        let mut w = World::new();
        w.spawn(0, 0);
        // Full throttle, hard lock: try to drive straight through a barrier.
        for _ in 0..60 * 30 {
            w.inputs[0] = CarInput {
                throttle: 1.0,
                steer: 1.0,
                brake: 0.0,
                handbrake: 0.0,
            };
            w.step(1);
            let hit = w.track.nearest(w.cars[0].pos(), w.cars[0].seg as u16);
            assert!(
                w.cars[0].lat.abs() < hit.half_width + 1.0,
                "escaped the track: lat {:.2} hw {:.2}",
                w.cars[0].lat,
                hit.half_width
            );
        }
    }

    #[test]
    fn static_cars_are_solid_but_immovable() {
        // The client-side arrangement: only car 0 is simulated, car 1 is a
        // network ghost that must still block.
        let mut w = World::new();
        w.spawn(0, 0);
        w.spawn(1, 1);
        let parked = w.cars[1];
        // Aim car 0 straight at car 1.
        let to = parked.pos().sub(w.cars[0].pos());
        w.cars[0].heading = math::atan2(to.y, to.x);
        for _ in 0..240 {
            w.inputs[0] = CarInput {
                throttle: 1.0,
                ..Default::default()
            };
            w.step(1); // only car 0 in the mask
        }
        assert_eq!(w.cars[1], parked, "a non-simulated car moved");
        let gap = w.cars[0].pos().sub(parked.pos()).len();
        assert!(gap > 1.5, "cars interpenetrated: {gap:.2} m apart");
    }

    /// Two cars nose to tail at a closing speed, staged on the start straight.
    fn shunt(closing: f32) -> World {
        let mut w = World::new();
        w.spawn(0, 0);
        w.spawn(1, 1);
        let (p, heading) = w.track.grid_slot(0);
        let fwd = V2::from_angle(heading);
        for (i, along, speed) in [(0usize, 0.0f32, closing + 20.0), (1, 6.0, 20.0)] {
            w.cars[i].place(p.add(fwd.scale(along)), heading);
            w.cars[i].active = 1.0;
            w.cars[i].vx = fwd.x * speed;
            w.cars[i].vy = fwd.y * speed;
            w.cars[i].sync_drivetrain();
            let hit = w.track.nearest(w.cars[i].pos(), track::NO_HINT);
            w.cars[i].seg = hit.idx as f32;
            w.cars[i].s = hit.s;
            w.cars[i].lat = hit.lat;
        }
        w
    }

    /// Two cars queued nose to tail on the racing line, `gap` metres apart
    /// along it, the one behind closing at `closing` m/s.
    ///
    /// Along the centreline rather than along a straight tangent, unlike
    /// [`shunt`], so the queue can be long enough that the client has taken
    /// delivery of a dozen snapshots before the contact. Without that the test
    /// below would be measuring a cold start rather than a prediction.
    fn queue(closing: f32, gap: f32) -> World {
        let mut w = World::new();
        w.spawn(0, 0);
        w.spawn(1, 1);
        let start = w.track.nearest(w.cars[0].pos(), track::NO_HINT).s;
        for (i, along, speed) in [(0usize, 0.0f32, closing + 20.0), (1, gap, 20.0)] {
            let (p, tan, _) = w.track.sample(w.track.wrap_s(start + along));
            w.cars[i].place(p, math::atan2(tan.y, tan.x));
            w.cars[i].active = 1.0;
            w.cars[i].vx = tan.x * speed;
            w.cars[i].vy = tan.y * speed;
            w.cars[i].sync_drivetrain();
            let hit = w.track.nearest(p, track::NO_HINT);
            w.cars[i].seg = hit.idx as f32;
            w.cars[i].s = hit.s;
            w.cars[i].lat = hit.lat;
        }
        w
    }

    /// Park a rival the way the browser used to.
    ///
    /// `Sim.placeRemote` was the whole of what a client would write into a car
    /// it does not own, so this is the list, and the list is load-bearing: the
    /// contact solver borrows a ghost car's velocity *and* its yaw rate for the
    /// length of a tick, and a contact point is `v + omega x r`. A rival parked
    /// without `omega` is a car that cornering does not rotate, which is worth
    /// over a metre a second of closing speed at the bodywork.
    fn park(ghost: &mut CarState, from: &CarState) {
        ghost.x = from.x;
        ghost.y = from.y;
        ghost.heading = from.heading;
        ghost.vx = from.vx;
        ghost.vy = from.vy;
        ghost.omega = from.omega;
        ghost.active = 1.0;
    }

    /// The record as it reaches the browser.
    ///
    /// `slip_f`, `slip_r`, `impact` and `wall` are the four fields of it that
    /// are not on the wire. Each is written before it is next read, so dropping
    /// them is exact rather than approximate -- and a test that quietly handed
    /// them over would be doing the client a favour it never gets.
    fn from_wire(c: &CarState) -> CarState {
        CarState { slip_f: 0.0, slip_r: 0.0, impact: 0.0, wall: 0.0, ..*c }
    }

    /// Carry a pose forward at a constant turn rate: what `predict()` in
    /// `net.ts` does for a car it has nothing better to go on for.
    fn extrapolate(c: &CarState, dt: f32) -> CarState {
        let w = c.omega;
        let th = w * dt;
        let straight = math::abs(w) < 1e-3;
        let s = if straight { dt } else { math::sin(th) / w };
        let k = if straight { 0.0 } else { (1.0 - math::cos(th)) / w };
        let (cs, sn) = (math::cos(th), math::sin(th));
        CarState {
            x: c.x + c.vx * s - c.vy * k,
            y: c.y + c.vx * k + c.vy * s,
            heading: c.heading + th,
            vx: c.vx * cs - c.vy * sn,
            vy: c.vx * sn + c.vy * cs,
            ..*c
        }
    }

    /// How a browser places the car it does not own.
    #[derive(Clone, Copy, PartialEq)]
    enum Rival {
        /// Parked at a constant-turn-rate extrapolation of its newest snapshot:
        /// solid, never integrated, and with no idea what its driver is doing.
        Parked,
        /// Seeded from that snapshot and stepped on the controls the authority
        /// published with it, held until the next one arrives.
        Stepped,
    }

    /// One approach: two cars closing, both drivers doing something.
    struct Scene {
        what: &'static str,
        closing: f32,
        gap: f32,
        /// What both drivers are doing with the wheel. The same, because they
        /// are going through the same piece of road.
        steer: f32,
        /// What the car in front does with its brake, given how much clear air
        /// is left behind it. Triggered on the gap rather than on the clock, so
        /// that "at the last moment" stays the last moment when the approach
        /// speed changes.
        brake: fn(f32) -> f32,
    }

    impl Scene {
        fn mine(&self) -> CarInput {
            CarInput { steer: self.steer, ..Default::default() }
        }
        fn rival(&self, gap: f32) -> CarInput {
            CarInput { steer: self.steer, brake: (self.brake)(gap), ..Default::default() }
        }
    }

    /// Ticks between snapshots, as the sidecar publishes them: 60 Hz / 3.
    const SNAP_EVERY: u64 = 3;
    /// Ticks the client is running ahead of the authority -- a 300 ms round
    /// trip, plus the margin the clock sync holds on top.
    ///
    /// The whole difficulty lives in this number. At zero lead every scheme is
    /// exact, because the snapshot *is* the answer; the question is only ever
    /// what to do with the ten ticks after it.
    const LEAD: u64 = 9;
    /// Ticks to keep running after the contact, so it has finished playing out
    /// before the answer is read.
    const SETTLE: i32 = 12;

    /// Run one approach twice: on the authority, and on a client `LEAD` ticks
    /// ahead of it taking the authority's snapshots at 20 Hz and placing the
    /// rival the given way.
    ///
    /// Returns the *worst* the client's account of its own car ever got, in
    /// metres and in m/s.
    ///
    /// The worst rather than the last, deliberately. A client that rolls back
    /// converges on the authority within a snapshot of anything: measure after
    /// the news has landed and every scheme scores zero, because what is being
    /// measured is then the correction rather than the prediction. The peak is
    /// what the driver actually experienced -- how far the car was from where
    /// the authority was going to say it was, at the moment they were looking
    /// at it -- and it is also, exactly, the size of the yank that followed.
    fn approach(sc: &Scene, style: Rival) -> (f32, f32) {
        let mut authority = queue(sc.closing, sc.gap);
        let mut client = queue(sc.closing, sc.gap);
        let mask = if style == Rival::Stepped { 0b11 } else { 0b01 };
        // Everything the authority has published, by tick: both cars, and the
        // controls the rival's pose was computed on. Only every third entry is
        // ever read, and keeping them all is simpler than a ring buffer.
        let opening = sc.rival(sc.gap);
        let mut published = vec![(client.cars[0], client.cars[1], opening)];
        if style == Rival::Stepped {
            client.inputs[1] = opening;
        }
        let mut applied = 0u64;
        let mut settling = -1i32;
        let (mut worst_pos, mut worst_speed) = (0.0f32, 0.0f32);

        for _ in 0..600 {
            let t = authority.tick;
            let held = sc.rival(authority.cars[1].pos().sub(authority.cars[0].pos()).len());
            authority.inputs[0] = sc.mine();
            authority.inputs[1] = held;
            authority.step(0b11);
            published.push((authority.cars[0], authority.cars[1], held));

            // The newest snapshot old enough to have arrived.
            let seen = authority.tick.saturating_sub(LEAD);
            let snap = seen - seen % SNAP_EVERY;
            if snap > applied {
                applied = snap;
                let (mine, theirs, held) = published[snap as usize];
                client.cars[0] = from_wire(&mine);
                client.tick = snap;
                if style == Rival::Stepped {
                    client.cars[1] = from_wire(&theirs);
                    client.inputs[1] = held;
                }
                // Rewind, and replay to where the client already was.
                for u in snap..t {
                    client.inputs[0] = sc.mine();
                    if style == Rival::Parked {
                        park(&mut client.cars[1], &extrapolate(&theirs, (u - snap) as f32 * DT));
                    }
                    client.step(mask);
                }
            }
            client.inputs[0] = sc.mine();
            if style == Rival::Parked {
                let theirs = published[applied as usize].1;
                park(&mut client.cars[1], &extrapolate(&theirs, (t - applied) as f32 * DT));
            }
            client.step(mask);

            // Both worlds are now at the same tick, and the authority's answer
            // for it will not reach the client for another [`LEAD`] ticks. The
            // gap between them is what the driver is looking at.
            let apart = client.cars[0].pos().sub(authority.cars[0].pos()).len();
            let dv = client.cars[0].forward_speed() - authority.cars[0].forward_speed();
            worst_pos = math::max(worst_pos, apart);
            worst_speed = math::max(worst_speed, math::abs(dv));

            if settling >= 0 {
                settling += 1;
                if settling >= SETTLE {
                    break;
                }
            } else if authority.cars[0].impact > 1.0 {
                settling = 0;
            }
        }
        assert!(settling >= 0, "{}: the two cars never touched", sc.what);
        (worst_pos, worst_speed)
    }

    /// A car-to-car impact is mostly plastic, and more so the harder it is.
    ///
    /// This is the difference between a race and a game of pool. The old model
    /// used a fixed coefficient of restitution, which made a 100 km/h shunt as
    /// springy as a 5 km/h one and sent both cars away from each other with
    /// speed they should have left in the bodywork.
    #[test]
    fn a_shunt_is_absorbed_rather_than_returned() {
        println!("  closing   separation   as a fraction   nose");
        let mut previous = 0.0f32;
        for closing in [4.0f32, 12.0, 24.0] {
            let mut w = shunt(closing);
            for _ in 0..40 {
                w.step(0b11);
            }
            let (a, b) = (w.cars[0].forward_speed(), w.cars[1].forward_speed());
            let fraction = (b - a) / closing;
            let nose = w.cars[0].dmg_front;
            println!(
                "  {closing:5.0} m/s   {:7.2} m/s   {:11.1} %   {nose:.3} m",
                b - a,
                fraction * 100.0
            );
            // Almost none of the closing speed comes back: it went into the
            // shape of both cars. How the fraction falls with speed is the
            // crush model's own claim and `collide` tests it directly; what
            // matters here is that a whole car, tires and all, agrees.
            assert!(fraction < 0.25, "{:.0}% of the closing speed came back", fraction * 100.0);
            assert!(nose > previous, "a harder shunt did no more damage: {nose:.3} m");
            previous = nose;
        }
    }

    /// The one a browser cares about: the client owns one car, gets the other
    /// from the network a round trip late, and has to arrive at the sidecar's
    /// answer for its own car anyway.
    ///
    /// The rival is *doing something* through the approach in most of these,
    /// which is the whole point. A car travelling in a straight line is a car
    /// any extrapolation can follow; a car braking, or leaning on the wheel, or
    /// lifting at the last moment, is not, and those are the moments a contact
    /// actually happens in. Both columns are printed because the improvement is
    /// the argument: the left is what parking a rival at an extrapolated pose
    /// got you, the right is what carrying it forward through the physics on
    /// the controls the authority published gets you instead.
    #[test]
    fn a_client_predicts_the_hit_the_authority_resolves() {
        let scenes = [
            Scene { what: "travelling", closing: 20.0, gap: 24.0, steer: 0.0, brake: |_| 0.0 },
            Scene { what: "braking hard", closing: 14.0, gap: 20.0, steer: 0.0, brake: |_| 1.0 },
            Scene { what: "leaning on the wheel", closing: 14.0, gap: 20.0, steer: 0.25, brake: |_| 0.0 },
            Scene { what: "braking mid-corner", closing: 12.0, gap: 18.0, steer: 0.25, brake: |_| 1.0 },
            // The one held input cannot know about until a snapshot says so,
            // and the honest limit of the scheme: for the ten ticks the news
            // takes to arrive the client is carrying a rival that is not
            // braking, because a tick ago it was not.
            Scene { what: "braking late", closing: 20.0, gap: 24.0, steer: 0.0, brake: |gap| if gap < 6.0 { 1.0 } else { 0.0 } },
        ];
        // A zero in the right-hand column is not a rounded zero. When a driver
        // holds an input -- which is what a driver mostly does -- the client
        // re-runs the authority's own arithmetic on the authority's own numbers
        // and lands on the same bits, so there is nothing left to be out by.
        // The last row is where the scheme actually costs something: a car that
        // changes its mind inside the round trip cannot be followed, only
        // corrected, and five centimetres is what that costs here.
        println!("  worst the client was ever out by, on its own car:");
        println!("  the rival is...            parked ghost        stepped rival");
        let mut worst = 0.0f32;
        for sc in &scenes {
            let (parked, parked_v) = approach(sc, Rival::Parked);
            let (stepped, stepped_v) = approach(sc, Rival::Stepped);
            println!(
                "  {:<24} {parked:5.2} m {parked_v:6.2} m/s   {stepped:5.2} m {stepped_v:6.2} m/s",
                sc.what
            );
            worst = math::max(worst, stepped);
            assert!(
                stepped < 0.25,
                "{}: client's car was {stepped:.2} m from the authority's",
                sc.what
            );
            assert!(
                stepped_v < 1.0,
                "{}: client was {stepped_v:.2} m/s off the authority through the hit",
                sc.what
            );
            // Not "better on average": better every time. An extrapolated pose
            // has no way to be right about a car whose driver is doing
            // anything, so there should be no scenario here it wins.
            assert!(
                stepped <= parked,
                "{}: stepping the rival was worse than parking it ({stepped:.2} m vs {parked:.2} m)",
                sc.what
            );
        }
        println!("  worst {worst:.3} m");
    }

    /// Damage has to be simulation state, not decoration: replaying from a
    /// snapshot that carried it must reproduce the same car.
    #[test]
    fn damage_survives_a_rollback() {
        let mut w = shunt(22.0);
        for _ in 0..30 {
            w.step(0b11);
        }
        let hurt = w.cars[0];
        let rival = w.cars[1];
        assert!(hurt.dmg_front > 0.05, "the setup did no damage");

        // Drive both on for a second, then replay the same second from the
        // saved state and demand the same bits.
        let mut inputs = Vec::new();
        for tick in 0..60u64 {
            inputs.push(scripted_input(tick));
        }
        let finish = |w: &mut World| {
            for inp in &inputs {
                w.inputs[0] = *inp;
                w.step(0b11);
            }
            w.cars[0]
        };
        let mut a = w;
        let first = finish(&mut a);

        let mut b = World::new();
        b.spawn(0, 0);
        b.spawn(1, 1);
        b.cars[0] = hurt;
        b.cars[1] = rival;
        b.tick = 30;
        let second = finish(&mut b);
        assert_eq!(first, second, "a replay from a damaged snapshot diverged");
    }

    /// Damage has to clear itself somewhere, or it is a one-way ratchet: a bot
    /// has no thumbs and cannot press *Respawn*, and neither can a driver who
    /// has not found the button. Completing a lap is where.
    #[test]
    fn completing_a_lap_repairs_the_car() {
        let mut w = World::new();
        w.spawn(0, 0);
        let brain = BotBrain::new(3);
        let wreck = damage::Damage { front: 0.30, rear: 0.10, left: 0.25, right: 0.05 };
        w.cars[0].set_damage(&wreck);
        let from = w.cars[0].lap;

        let mut carried = 0u64;
        let mut lapped = 0u64;
        for tick in 0..60 * 150u64 {
            w.inputs[0] = brain.drive(0, &w.cars, w.active, &w.track, tick);
            w.step(1);
            if w.cars[0].lap > from {
                lapped = tick;
                break;
            }
            if w.cars[0].damage().severity() > 0.5 {
                carried += 1;
            }
        }
        println!(
            "wrecked to {:.2}, lapped after {:.1} s, carried it for {:.1} s, finished at {:.2}",
            wreck.severity(),
            lapped as f32 * DT,
            carried as f32 * DT,
            w.cars[0].damage().severity()
        );
        assert!(lapped > 0, "the bot never completed a lap");
        // Long enough that it is a consequence and not a formality.
        assert!(carried as f32 * DT > 20.0, "the damage was gone before it mattered");
        assert_eq!(w.cars[0].damage(), damage::Damage::default(), "the car was not repaired");
    }

    #[test]
    fn layout_matches_the_wasm_bridge_contract() {
        assert_eq!(std::mem::size_of::<CarState>(), CAR_FLOATS * 4);
        assert_eq!(std::mem::size_of::<CarInput>(), INPUT_FLOATS * 4);
        assert_eq!(std::mem::align_of::<CarState>(), 4);
    }
}
