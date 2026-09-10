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
        assert!(c.speed() < 5.0, "still doing {:.1} m/s after a spin", c.speed());
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

    /// The one a browser cares about: the client owns one car and takes the
    /// other from the network, and it has to arrive at the sidecar's answer for
    /// its own car anyway.
    ///
    /// It did not used to. A client cannot move a car it does not own, so it
    /// gave rivals infinite mass -- and an infinite mass returns the entire
    /// impulse. You rebounded off a car you should have shoved, and then the
    /// authority's snapshot dragged you back through it.
    #[test]
    fn a_client_predicts_the_hit_the_authority_resolves() {
        println!("  closing   authority   client   error");
        for closing in [6.0f32, 20.0] {
            let mut authority = shunt(closing);
            let mut client = shunt(closing);
            let mut hit_at = -1i32;
            while hit_at < 12 {
                authority.step(0b11);
                // The rival's snapshot for this tick. `net.ts` interpolates
                // remote cars up to the current tick, so this is what a client
                // really has, not a favour to it.
                client.cars[1] = authority.cars[1];
                client.step(0b01);
                if hit_at >= 0 {
                    hit_at += 1;
                } else if authority.cars[0].impact > 1.0 {
                    hit_at = 0;
                }
            }
            let (a, c) = (authority.cars[0].forward_speed(), client.cars[0].forward_speed());
            let apart = client.cars[0].pos().sub(authority.cars[0].pos()).len();
            println!("  {closing:5.0} m/s   {a:6.2} m/s   {c:5.2} m/s   {:.2} m/s, {apart:.3} m", c - a);
            assert!(
                math::abs(c - a) < 1.5,
                "client predicted {c:.2} m/s where the authority resolved {a:.2}"
            );
            assert!(apart < 0.5, "client's car ended up {apart:.2} m from the authority's");
        }
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
