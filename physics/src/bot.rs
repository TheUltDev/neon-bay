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
    /// Grip the driver *plans* for. Below the car's real limit on purpose --
    /// the margin is what stops every apex being reached sideways.
    pub plan_grip: f32,
    /// Yaw-rate feedback gain. Higher is twitchier but catches slides sooner.
    pub yaw_gain: f32,
}

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
            plan_grip: 1.35,
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
        let grip = self.plan_grip * 1.15 * self.skill;
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
        let steer = clamp(delta / crate::car::steer_lock(speed), -1.0, 1.0);

        // --- speed ---------------------------------------------------------
        // Assume a bit less grip than the car really has -- the margin is what
        // keeps a bot from arriving at every apex already sideways.
        let plan_grip = self.plan_grip * self.skill;
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

        // Recovery. A spun-out bot that keeps chasing a look-ahead point 40 m
        // away just spins faster, so take over and point it down the road.
        let (_, tan_here, _) = track.sample(car.s + 6.0);
        let facing_err = wrap_pi(atan2(tan_here.y, tan_here.x) - car.heading);
        if speed < 9.0 && abs(facing_err) > 0.55 {
            if abs(facing_err) > 2.0 {
                return CarInput {
                    throttle: -0.7,
                    steer: clamp(-facing_err * 0.9, -1.0, 1.0),
                    brake: 0.0,
                    handbrake: 0.0,
                };
            }
            return CarInput {
                throttle: 0.42,
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
