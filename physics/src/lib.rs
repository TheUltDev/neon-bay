//! Deterministic top-down car simulation, compiled twice from one source:
//!
//! * as an `rlib` linked into the **sidecar**, which is the authority;
//! * as a `cdylib` for `wasm32-unknown-unknown`, loaded by the **browser**,
//!   which predicts the local car and rolls back when the authority disagrees.
//!
//! Because both sides run the same instructions on the same inputs, prediction
//! error on a healthy connection is exactly zero, and the reconciliation code
//! only has to earn its keep when packets are late, dropped or tampered with.

pub mod bot;
pub mod car;
pub mod ffi;
pub mod math;
pub mod track;
pub mod world;

pub use car::{CarInput, CarState, CAR_FLOATS, DT, INPUT_FLOATS, TICK_HZ};
pub use math::V2;
pub use track::{Track, CHECKPOINTS, SAMPLES};
pub use world::{World, MAX_CARS};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bot::BotBrain;

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
        // Free space: just the drivetrain, no track and no walls.
        let mut c = CarState::default();
        let inp = CarInput {
            throttle: 1.0,
            ..Default::default()
        };
        let mut at_3s = 0.0f32;
        for i in 0..60 * 60 {
            for _ in 0..car::SUBSTEPS {
                car::integrate(&mut c, &inp, car::H);
            }
            if i == 180 {
                at_3s = c.forward_speed();
            }
        }
        let top = c.forward_speed();
        println!(
            "0-3s {:.1} m/s, top {:.1} m/s ({:.0} km/h)",
            at_3s,
            top,
            top * 3.6
        );
        assert!(at_3s > 22.0, "0-3 s only reached {at_3s:.1} m/s");
        assert!(top > 55.0 && top < 90.0, "top speed {top:.1} m/s");
    }

    #[test]
    fn chassis_pulls_real_lateral_g_and_understeers_at_the_limit() {
        // Steady-state skidpad. The car should push wide as grip runs out --
        // an oversteer-limited car is undriveable with a keyboard.
        let corner = |steer: f32| {
            let mut c = CarState::default();
            c.vx = 30.0;
            for _ in 0..1200 {
                let err = 30.0 - c.speed();
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
            let fwd = c.forward_speed();
            ((c.omega * fwd).abs() / 9.81, c.slip_f, c.slip_r)
        };
        let (g_mid, _, _) = corner(0.4);
        let (g_max, slip_f, slip_r) = corner(0.8);
        println!("skidpad: 0.4 -> {g_mid:.2} g, 0.8 -> {g_max:.2} g (slip f {slip_f:.3} r {slip_r:.3})");
        assert!(g_mid > 1.3, "only {g_mid:.2} g available");
        assert!(g_max < g_mid * 1.15, "lateral g still climbing at full lock");
        assert!(
            slip_f.abs() > slip_r.abs(),
            "car oversteers at the limit (front {slip_f:.3} rear {slip_r:.3})"
        );
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

    #[test]
    fn layout_matches_the_wasm_bridge_contract() {
        assert_eq!(std::mem::size_of::<CarState>(), CAR_FLOATS * 4);
        assert_eq!(std::mem::size_of::<CarInput>(), INPUT_FLOATS * 4);
        assert_eq!(std::mem::align_of::<CarState>(), 4);
    }
}
