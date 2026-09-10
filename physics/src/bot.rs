//! Bot driver: pure pursuit on the centerline with an apex-seeking offset and
//! a curvature look-ahead for corner entry speed.
//!
//! Bots exist to make the point that the sidecar is doing real work. Their AI
//! runs *only* there -- the browser never simulates them, it interpolates the
//! poses the authority publishes.

use crate::car::{CarInput, CarState};
use crate::math::{abs, atan2, clamp, wrap_pi};
use crate::track::Track;

#[derive(Clone, Copy, Debug)]
pub struct BotBrain {
    /// 0.85..1.05 scale on cornering speed. Below 1 is a cautious driver.
    pub skill: f32,
    /// Preferred lateral bias, meters. Gives each bot a personality.
    pub line_bias: f32,
    /// Phase offset so the wobble of two bots never lines up.
    pub phase: f32,
    /// Fraction of the car's *real* limit the driver plans for.
    ///
    /// It used to be an absolute number of g, which only worked while the car
    /// had a fixed amount of grip. It no longer does: downforce adds grip with
    /// speed and load sensitivity takes some back in the transfer, so the
    /// driver asks [`crate::car::grip_limit`] what is actually available and
    /// keeps this much of it. The rest is the margin that stops every apex
    /// being arrived at sideways.
    pub plan_grip: f32,
    /// Yaw-rate feedback gain. Higher is twitchier but catches slides sooner.
    pub yaw_gain: f32,
}

/// Ticks the recovery drives each way before trying the other one.
///
/// A second and a quarter, and it needs to be: a gear change is 120 ms and the
/// clutch takes as long again to come back in, so a shorter phase spends most
/// of itself waiting for the drivetrain and never gets a shove out of it.
const ROCK_TICKS: u64 = 75;

impl BotBrain {
    pub fn new(seed: u32) -> Self {
        // Cheap deterministic hash -> three personality knobs.
        let mut h = seed.wrapping_mul(0x9e37_79b9) ^ 0x85eb_ca6b;
        let mut next = || {
            h ^= h << 13;
            h ^= h >> 17;
            h ^= h << 5;
            (h & 0xffff) as f32 / 65535.0
        };
        BotBrain {
            skill: 0.86 + next() * 0.19,
            line_bias: (next() - 0.5) * 3.0,
            phase: next() * 6.28,
            plan_grip: 0.60,
            yaw_gain: 0.16,
        }
    }

    pub fn drive(
        &self,
        me: usize,
        cars: &[CarState],
        active: u32,
        track: &Track,
        tick: u64,
    ) -> CarInput {
        let car = cars[me];
        let speed = car.speed();
        let fwd_speed = car.forward_speed();

        // --- where do I want to be, ~1 second up the road? ----------------
        let look = clamp(9.0 + speed * 0.72, 9.0, 42.0);
        let (_, _, curv_here) = track.sample(car.s + look * 0.35);
        let (aim_p, aim_t, _) = track.sample(car.s + look);
        let n = aim_t.perp();

        // Hug the inside of the corner, drift out on the exit.
        let apex = clamp(curv_here * 110.0, -1.0, 1.0) * 4.6;
        let wobble = crate::math::sin(tick as f32 * 0.011 + self.phase) * 0.7;
        let mut offset = apex + self.line_bias + wobble;

        // --- avoid whoever is directly in front ---------------------------
        for (j, other) in cars.iter().enumerate() {
            if j == me || active & (1 << j) == 0 {
                continue;
            }
            let rel = other.pos().sub(car.pos());
            let d = rel.len();
            if d > 18.0 || d < 0.01 {
                continue;
            }
            let local = rel.to_local(car.heading);
            if local.x < 1.0 {
                continue; // behind or alongside
            }
            let closeness = 1.0 - d / 18.0;
            // Steer for the side they are not on.
            let side = if local.y >= 0.0 { -1.0 } else { 1.0 };
            offset += side * closeness * 5.0;
        }

        let hw = track.nearest(aim_p, car.seg as u16).half_width;
        offset = clamp(offset, -(hw - 2.2), hw - 2.2);
        let target = aim_p.add(n.scale(offset));

        // --- steering ------------------------------------------------------
        // Pure pursuit gives a desired yaw rate; that gets clipped to what the
        // tires can actually deliver and converted back into a steering angle.
        // The yaw-rate feedback term doubles as automatic opposite lock: in a
        // slide the measured rate overshoots the command and the wheel unwinds.
        // What this car can really do at this speed, in g. Rises with
        // downforce, which is why a bot carries more speed through a fast
        // sweeper than through a hairpin of the same radius.
        let limit = crate::car::grip_limit(speed, &car.damage()) / (crate::car::MASS * crate::car::G);
        let plan_grip = limit * self.plan_grip * self.skill;
        let grip = plan_grip * 1.15;
        let to_target = target.sub(car.pos());
        let dist = if to_target.len() > 5.0 { to_target.len() } else { 5.0 };
        let local = to_target.to_local(car.heading);
        let alpha = atan2(local.y, local.x);
        let v = if fwd_speed > 5.0 { fwd_speed } else { 5.0 };
        let omega_des = 2.0 * v * crate::math::sin(alpha) / dist;
        let omega_max = grip * 9.81 / v;
        let omega_cmd = clamp(omega_des, -omega_max, omega_max);
        let delta = crate::math::atan(crate::car::WHEELBASE * omega_cmd / v)
            + (omega_cmd - car.omega) * self.yaw_gain;
        // Back out the input that produces that road-wheel angle *in this car*.
        // A damaged one has less lock to give and a permanent pull built into
        // its geometry, and a driver who does not hold against the pull keeps
        // arriving back at the barrier that put it there -- which is a spiral,
        // not a consequence. This is the same arithmetic `car::integrate` does
        // on the way in, run backwards.
        let dmg = car.damage();
        let lock = crate::car::steer_lock(speed) * dmg.steer_lock();
        let steer = clamp((delta - dmg.steer_pull()) / lock, -1.0, 1.0);

        // --- speed ---------------------------------------------------------
        let horizon = clamp(50.0 + speed * 2.6, 50.0, 240.0);
        let target_speed = clamp(track.speed_limit(car.s, horizon, plan_grip), 7.0, 80.0);

        let err = target_speed - fwd_speed;
        let (mut throttle, brake) = if err > 0.6 {
            (clamp(err * 0.35, 0.25, 1.0), 0.0)
        } else if err < -1.2 {
            // Ease off the brakes the more lock is wound on.
            (0.0, clamp(-err * 0.16, 0.15, 1.0) * (1.0 - 0.5 * abs(steer)))
        } else {
            (clamp(0.35 + err * 0.2, 0.0, 0.6), 0.0)
        };

        // Traction management: a real driver does not stamp on it at 20 km/h
        // with the wheel crossed up, and lifts when the back steps out.
        let by_speed = clamp(0.42 + speed * 0.038, 0.42, 1.0);
        let by_steer = 1.0 - 0.5 * abs(steer);
        // Slip angle is noise below walking pace, so only trust it once moving.
        let slide_trust = clamp((speed - 6.0) / 6.0, 0.0, 1.0);
        let by_slide = 1.0 - clamp((abs(car.slip_r) - 0.14) * 3.2, 0.0, 0.85) * slide_trust;
        throttle = clamp(throttle, 0.0, by_speed * by_steer * by_slide);

        // --- recovery -------------------------------------------------------
        // A spun-out bot that keeps chasing a look-ahead point 40 m away just
        // spins faster, and one wedged against a barrier will sit there
        // spinning its wheels for the rest of the race. Both need the driver
        // taken away from it.
        let (_, tan_here, _) = track.sample(car.s + 6.0);
        let facing_err = wrap_pi(atan2(tan_here.y, tan_here.x) - car.heading);

        // Stopped *and* actually in contact with something. `wall` is set by
        // the collision solver when a corner of the body is outside the
        // barrier, so this is the car reporting a contact rather than the
        // driver inferring one from how wide it is running -- which fires on
        // any bot taking a normal wide line and pitches it into reverse.
        let wedged = speed < 3.0 && car.wall > 0.5;
        let reversing = fwd_speed < -0.5;
        // Hysteresis on the threshold, so a car part way through backing out
        // does not change its mind and drive into the barrier again.
        let too_far_round = abs(facing_err) > if reversing { 0.8 } else { 1.4 };

        if wedged || (speed < 9.0 && abs(facing_err) > 0.55) {
            // A car that is genuinely pinned may be pinned in exactly the
            // direction it is pointing, and backing out of a barrier it is
            // wedged into can drive it further in. So rock it, which is what a
            // person does with a car that will not come free, and which needs
            // no memory to run -- the tick counter is the same on the
            // authority and in the browser.
            //
            // The clock only gets a say while the car is genuinely stopped.
            // Two things went wrong when it had a wider one. It used to be
            // `wedged` that chose between the clock and the compass, and
            // `wedged` reads a contact flag that goes on and off between one
            // tick and the next as a corner rests on the barrier line, so the
            // driver changed its mind several times a second -- which rocks a
            // car at a frequency that builds no speed at all. And asking for
            // reverse from a car that is already rolling forwards achieves
            // nothing whatever: the gearbox will not select it at speed and a
            // negative pedal is not a negative torque, so the car coasts, stays
            // under the threshold that got it here, and is asked again. Either
            // way a bot could spend a whole race travelling four metres.
            let back = if speed < 0.8 {
                (tick / ROCK_TICKS) % 2 == 0
            } else {
                too_far_round
            };
            if back {
                // Back out properly. A third of a pedal used to seem like the
                // careful thing to ask for, on the grounds that reverse is
                // geared short enough to light the rear tires up -- but it is
                // not enough to lift the engine off idle against a clutch that
                // is barely engaged at idle, so the car creeps at a fifth of a
                // metre a second and never gets anywhere. Traction control is
                // direction-aware and will take back whatever is too much.
                // Steering is negated because reversing swings the nose the
                // other way.
                return CarInput {
                    throttle: -0.75,
                    steer: clamp(-facing_err * 0.9, -1.0, 1.0),
                    brake: 0.0,
                    handbrake: 0.0,
                };
            }
            return CarInput {
                throttle: 0.6,
                steer: clamp(facing_err * 2.2, -1.0, 1.0),
                brake: 0.0,
                handbrake: 0.0,
            };
        }

        CarInput {
            throttle,
            steer,
            brake,
            handbrake: 0.0,
        }
    }
}
