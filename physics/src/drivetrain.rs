//! Engine, clutch, gearbox and differential: everything between the throttle
//! pedal and the two rear contact patches.
//!
//! The old model multiplied throttle by a force and called it a drivetrain.
//! This one carries the engine's rotational speed as real state and lets the
//! road push back on it, which changes the car in ways worth having:
//!
//! * **Torque follows the curve, not the pedal.** [`TORQUE`] peaks in the
//!   middle of the range and falls off at both ends, so the same throttle
//!   opening does different things at 2000 rpm and at 7000, and being in the
//!   wrong gear costs you real time.
//! * **The engine has inertia.** Spinning it up absorbs torque that is then not
//!   reaching the road, which is most of why first gear is slower than the
//!   gearing suggests.
//! * **The clutch slips.** [`cap`] limits what it can carry as a function of
//!   engine speed, so pulling away flares the revs and lets them bite -- and
//!   the engine can never be dragged below idle and stalled.
//! * **The differential is not a solid axle.** A limited-slip unit sends torque
//!   away from a wheel that is spinning up, which is what lets a rear-drive car
//!   put power down on corner exit with the inside wheel light.
//!
//! Gear selection is automatic because the client sends a throttle axis and not
//! a shifter. The shift itself takes [`SHIFT_TIME`] with the clutch out, so an
//! upshift mid-corner really does unload the rear for a moment.

use crate::math::{abs, clamp, max, signum};

/// Radians per second per rpm.
pub const RPM_TO_RAD: f32 = 0.104_719_75;

/// Engine torque, N.m, sampled every 800 rpm from 0 to 8000. Peak torque of
/// 425 N.m at 4800 rpm; peak power of about 275 kW just under the limiter.
const TORQUE: [f32; 11] = [
    0.0, 250.0, 310.0, 350.0, 385.0, 410.0, 425.0, 420.0, 400.0, 365.0, 300.0,
];
/// Spacing of the samples above, in rad/s.
const TORQUE_STEP: f32 = 800.0 * RPM_TO_RAD;

/// Engine and flywheel rotational inertia.
const I_ENGINE: f32 = 0.22;

pub const IDLE_RPM: f32 = 900.0;
pub const REDLINE_RPM: f32 = 7600.0;
/// Fuel cut. Slightly under the redline so the needle never pins.
const LIMITER: f32 = 7400.0 * RPM_TO_RAD;
const IDLE: f32 = IDLE_RPM * RPM_TO_RAD;
/// The engine is never allowed below this, which is what "it cannot stall"
/// means in practice. The clutch releasing at [`CLUTCH_LO`] is what makes it
/// true rather than enforced.
const STALL_FLOOR: f32 = 400.0 * RPM_TO_RAD;
/// Idle governor: a little torque, proportional to how far under idle it is.
const IDLE_GAIN: f32 = 3.0;
const IDLE_MAX: f32 = 90.0;

/// Closed-throttle drag: pumping losses and friction, rising with speed.
const BRAKE_BASE: f32 = 20.0;
const BRAKE_RATE: f32 = 0.055;

/// Forward gears, then the final drive.
///
/// Roughly geometric, each ratio about 0.73 of the one below it, which is what
/// keeps the engine landing back in the same part of its torque curve after
/// every shift. First is short enough to be over with by 60 km/h -- the
/// previous set ran it to 74 km/h, which meant a standing start spent three
/// seconds in one gear. Sixth is an overdrive tall enough that the limiter in
/// top sits past the speed the drag in [`crate::aero`] allows, so top speed is
/// set by power against air and not by running out of gearbox.
pub const GEARS: [f32; 6] = [3.55, 2.60, 1.91, 1.40, 1.03, 0.76];
pub const FINAL_DRIVE: f32 = 4.10;
const REVERSE: f32 = -3.60;

/// Road speed below which the gearbox will change direction, m/s.
///
/// A real one refuses at any more than a crawl, and is right to: a reverse
/// ratio is short enough that engaging it at speed would put the engine well
/// past the limiter backwards. [`crate::car::reverse_assist`] is what gets the
/// car down to here in the first place.
pub const REVERSE_BELOW: f32 = 0.6;

/// Driveline efficiency. The missing 8% is heat in the gears and bearings.
const EFFICIENCY: f32 = 0.92;

/// Shift points. Up near peak power, down far enough below it that a shift can
/// never immediately undo itself -- `gearbox_never_hunts` proves that for every
/// ratio pair rather than trusting the arithmetic here.
const UP_SHIFT: f32 = 7250.0 * RPM_TO_RAD;
const DOWN_SHIFT: f32 = 3300.0 * RPM_TO_RAD;
/// Seconds the clutch is out for. Long enough to feel, short enough to race.
pub const SHIFT_TIME: f32 = 0.12;
/// How fast the clutch comes back in afterwards, in engagement per second.
const CLUTCH_RATE: f32 = 8.0;

/// Torque the clutch can hold when fully engaged and spinning freely.
const CLUTCH_CAP: f32 = 700.0;
/// Stiffness of the slipping clutch, N.m per rad/s of slip. High enough that a
/// "locked" clutch gives up only tens of rpm under full load; the integrator
/// takes it implicitly, so the stiffness costs nothing in stability.
const CLUTCH_K: f32 = 90.0;
/// The clutch carries nothing below `LO` and everything above `HI`.
///
/// `LO` sits just *above* idle, which does two jobs: the engine can always
/// escape a load by dropping below it, so it cannot be stalled, and a car
/// sitting at idle in gear does not creep. The span up to `HI` is what makes a
/// standing start flare the revs and then bite, instead of bogging -- the
/// engine settles where its torque equals what the clutch can hold, which
/// `engine_revs_against_a_stalled_car` pins at a realistic launch speed. `HI`
/// is below the gearbox's downshift point, so under power the clutch is always
/// fully locked and none of this costs the car any torque.
const CLUTCH_LO: f32 = 950.0 * RPM_TO_RAD;
const CLUTCH_HI: f32 = 3_400.0 * RPM_TO_RAD;

/// Limited-slip differential. Preload is always there; the ramps add lock in
/// proportion to the torque going through it, and there is more of it on power
/// than on the overrun so that lifting mid-corner does not tie the axle
/// together and push the car straight on.
const LSD_PRELOAD: f32 = 90.0;
const LSD_RAMP_POWER: f32 = 0.32;
const LSD_RAMP_COAST: f32 = 0.14;
/// How hard the unit resists a speed difference, N.m per rad/s.
const LSD_K: f32 = 60.0;

/// What the drivetrain hands to the rear axle this substep.
#[derive(Clone, Copy, Debug, Default)]
pub struct Output {
    pub torque_l: f32,
    pub torque_r: f32,
    /// `-d(wheel torque) / d(wheel speed)`, which the wheel integrator folds
    /// into its implicit step. Without it a low gear couples a 0.22 kg.m^2
    /// engine to a 1.2 kg.m^2 wheel hard enough to blow up at any sane rate.
    pub coupling: f32,
    /// Engine speed as a fraction of the redline, for the tachometer.
    pub rpm: f32,
}

/// Everything between the pedal and the axle that has to be remembered from one
/// tick to the next.
#[derive(Clone, Copy, Debug)]
pub struct Drivetrain {
    /// Engine speed, rad/s.
    pub engine: f32,
    /// -1 reverse, 1..=6 forward.
    pub gear: f32,
    /// Seconds remaining of the current shift; 0 when engaged.
    pub shift: f32,
    /// Clutch engagement, 0..1.
    pub clutch: f32,
}

impl Default for Drivetrain {
    fn default() -> Self {
        Drivetrain {
            engine: IDLE,
            gear: 1.0,
            shift: 0.0,
            clutch: 1.0,
        }
    }
}

/// Engine torque at a given speed and throttle opening. Blends the wide-open
/// curve against closed-throttle drag, which is how engine braking arrives.
pub fn engine_torque(omega: f32, throttle: f32) -> f32 {
    // Fuel cut on the limiter: only the drag term survives.
    let open = if omega >= LIMITER { 0.0 } else { clamp(throttle, 0.0, 1.0) };

    let t = omega / TORQUE_STEP;
    let i = clamp(t, 0.0, (TORQUE.len() - 1) as f32) as usize;
    let j = if i + 1 < TORQUE.len() { i + 1 } else { i };
    let frac = clamp(t - i as f32, 0.0, 1.0);
    let wot = TORQUE[i] + (TORQUE[j] - TORQUE[i]) * frac;

    let drag = -(BRAKE_BASE + BRAKE_RATE * omega);
    let idle_assist = clamp((IDLE - omega) * IDLE_GAIN, 0.0, IDLE_MAX);

    open * wot + (1.0 - open) * drag + idle_assist
}

/// Torque the clutch can hold at this engine speed.
#[inline]
fn cap(engine: f32) -> f32 {
    CLUTCH_CAP * clamp((engine - CLUTCH_LO) / (CLUTCH_HI - CLUTCH_LO), 0.0, 1.0)
}

/// Overall ratio from the engine to the wheels, sign included.
#[inline]
fn ratio(gear: f32) -> f32 {
    if gear < 0.0 {
        REVERSE * FINAL_DRIVE
    } else {
        let i = clamp(gear, 1.0, GEARS.len() as f32) as usize - 1;
        GEARS[i] * FINAL_DRIVE
    }
}

/// The gear a driver accelerating through `speed` would be in, and the engine
/// speed that implies.
///
/// Needed whenever a car is handed a road speed without a drivetrain to match
/// -- a test setting up a corner, or a pose written in from outside. Getting it
/// wrong is not a cosmetic problem: leave a car in first at 126 km/h and the
/// engine is being driven at 15,000 rpm, the clutch hands the rear axle its
/// full capacity backwards through a ratio of fourteen, and the back end snaps
/// round before the gearbox can shift out of it. Which is exactly what a real
/// car would do, and exactly not what the caller meant.
pub fn gear_for(speed: f32) -> (f32, f32) {
    let w_wheel = abs(speed) / crate::wheel::RADIUS;
    let mut gear = GEARS.len();
    for (i, g) in GEARS.iter().enumerate() {
        if w_wheel * g * FINAL_DRIVE <= UP_SHIFT {
            gear = i + 1;
            break;
        }
    }
    let engine = clamp(
        w_wheel * GEARS[gear - 1] * FINAL_DRIVE,
        IDLE,
        REDLINE_RPM * RPM_TO_RAD,
    );
    (gear as f32, engine)
}

impl Drivetrain {
    /// Advance the engine and gearbox by `h` seconds and split the result
    /// across the two driven wheels.
    ///
    /// `w_rl` and `w_rr` are the rear wheel speeds in rad/s, `forward` the
    /// car's signed forward speed (only used to decide about reverse).
    pub fn step(&mut self, throttle: f32, w_rl: f32, w_rr: f32, forward: f32, h: f32) -> Output {
        // A `CarState` that came from `Default` -- an empty slot, or a snapshot
        // written in from outside -- has a stopped engine in no gear. Rather
        // than let every caller remember to prime it, normalise here.
        if self.gear == 0.0 {
            self.gear = 1.0;
        }
        if self.engine < STALL_FLOOR {
            self.engine = IDLE;
        }

        // --- gear selection ------------------------------------------------
        if self.shift > 0.0 {
            self.shift = max(self.shift - h, 0.0);
        } else if self.clutch > 0.9 {
            // Only decide once the clutch is properly back in. Deciding while
            // it is still feeding would read an engine speed that the road is
            // not yet connected to, and shift again on the strength of it.
            self.select(throttle, forward);
        }

        // The clutch comes out for a shift and eases back in afterwards.
        let target = if self.shift > 0.0 { 0.0 } else { 1.0 };
        let step = CLUTCH_RATE * h;
        self.clutch = clamp(self.clutch + clamp(target - self.clutch, -step, step), 0.0, 1.0);

        // Torque is cut for the duration of the shift, the way every automated
        // gearbox does it. Without the cut, a disconnected engine at full
        // throttle simply revs to the limiter during the 120 ms it is out of
        // gear -- and arrives back over the upshift threshold, which is a
        // gearbox that goes second, fourth, sixth in half a second.
        //
        // The sign is the gearbox's business, not the engine's: an engine only
        // ever turns one way, and `-1` on the wire means "reverse", not "run
        // the crankshaft backwards". Reverse is a negative *ratio* below, and
        // the pedal here is how far it is pressed.
        let pedal = if self.shift > 0.0 {
            0.0
        } else if self.gear < 0.0 {
            -throttle
        } else {
            throttle
        };

        // --- clutch --------------------------------------------------------
        let r = ratio(self.gear);
        let w_diff = (w_rl + w_rr) * 0.5;
        // Engine speed the road is currently demanding.
        let w_in = w_diff * r;
        let held = cap(self.engine) * self.clutch;
        let t_clutch = clamp(CLUTCH_K * self.clutch * (self.engine - w_in), -held, held);

        // --- engine --------------------------------------------------------
        // Implicit in the clutch slope: at 90 N.m per rad/s against 0.22 kg.m^2
        // an explicit step would need ~2.4 kHz to stay still.
        let t_eng = engine_torque(self.engine, pedal);
        self.engine += h * (t_eng - t_clutch) / (I_ENGINE + h * CLUTCH_K * self.clutch);
        self.engine = clamp(self.engine, STALL_FLOOR, REDLINE_RPM * RPM_TO_RAD);

        // --- differential ---------------------------------------------------
        let t_diff = t_clutch * r * EFFICIENCY;
        let base = t_diff * 0.5;

        // Power or overrun? The ramps differ, so the unit locks harder under
        // acceleration than it does off the throttle.
        let driving = if abs(w_diff) > 0.5 {
            signum(t_diff) == signum(w_diff)
        } else {
            true
        };
        let ramp = if driving { LSD_RAMP_POWER } else { LSD_RAMP_COAST };
        let lock_cap = LSD_PRELOAD + ramp * abs(t_diff);
        let lock = clamp(LSD_K * (w_rl - w_rr), -lock_cap, lock_cap);

        Output {
            torque_l: base - lock,
            torque_r: base + lock,
            // Both the clutch (through the gearing, squared) and the diff resist
            // a change in wheel speed.
            coupling: CLUTCH_K * self.clutch * r * r * EFFICIENCY * 0.25 + LSD_K,
            rpm: self.engine / (REDLINE_RPM * RPM_TO_RAD),
        }
    }

    /// Pick a gear. Automatic, because the wire carries a throttle axis and not
    /// a shifter.
    fn select(&mut self, throttle: f32, forward: f32) {
        // Reverse is only available from a near-standstill, in both directions.
        if throttle < -0.02 && forward < REVERSE_BELOW && self.gear > 0.0 {
            self.gear = -1.0;
            self.shift = SHIFT_TIME;
            return;
        }
        if self.gear < 0.0 {
            if throttle > -0.02 && forward > -REVERSE_BELOW {
                self.gear = 1.0;
                self.shift = SHIFT_TIME;
            }
            return;
        }

        let top = GEARS.len() as f32;
        if self.engine > UP_SHIFT && self.gear < top && throttle > 0.1 {
            self.gear += 1.0;
            self.shift = SHIFT_TIME;
        } else if self.engine < DOWN_SHIFT && self.gear > 1.0 {
            self.gear -= 1.0;
            self.shift = SHIFT_TIME;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Gearing has to put a usable road speed in every gear, and let top gear
    /// out past what the car can actually reach.
    #[test]
    fn the_ratios_cover_the_speed_range() {
        let wheel_speed = |gear: usize, engine: f32| {
            engine / (GEARS[gear] * FINAL_DRIVE) * crate::wheel::RADIUS
        };
        let first = wheel_speed(0, UP_SHIFT);
        let top = wheel_speed(GEARS.len() - 1, LIMITER);
        println!(
            "first gear runs to {:.0} km/h, sixth to {:.0} km/h on the limiter",
            first * 3.6,
            top * 3.6
        );
        assert!(first * 3.6 > 45.0 && first * 3.6 < 70.0, "first gear tops at {:.0} km/h", first * 3.6);
        // Comfortably past the ~76 m/s the drag in `aero.rs` allows, so the
        // gearbox is never what limits top speed.
        assert!(top > 79.0, "sixth gear caps the car at {:.0} m/s", top);
    }

    /// The engine must not be able to rev itself back over the upshift point
    /// while it is disconnected, or one shift triggers the next.
    #[test]
    fn a_shift_cuts_torque_so_it_cannot_chain() {
        let mut d = Drivetrain::default();
        d.gear = 2.0;
        d.engine = UP_SHIFT * 0.99;
        d.shift = SHIFT_TIME;
        d.clutch = 0.0;
        // Full throttle, wheels turning as they would be at that speed.
        let w = d.engine / (GEARS[1] * FINAL_DRIVE);
        let mut ticks = 0;
        while d.shift > 0.0 && ticks < 480 {
            d.step(1.0, w, w, w * crate::wheel::RADIUS, 1.0 / 480.0);
            ticks += 1;
        }
        println!(
            "engine through a full-throttle shift: {:.0} -> {:.0} rpm (upshift at {:.0})",
            UP_SHIFT * 0.99 / RPM_TO_RAD,
            d.engine / RPM_TO_RAD,
            UP_SHIFT / RPM_TO_RAD
        );
        assert_eq!(d.gear, 2.0, "gearbox shifted again mid-shift");
        assert!(d.engine < UP_SHIFT, "engine flared past the upshift point while out of gear");
    }

    /// A shift must never put the engine straight back over the opposite
    /// threshold, or the gearbox oscillates between two ratios forever.
    #[test]
    fn gearbox_never_hunts() {
        for i in 0..GEARS.len() - 1 {
            let after_up = UP_SHIFT * (GEARS[i + 1] / GEARS[i]);
            println!(
                "{}->{}: engine falls to {:.0} rpm (downshift at {:.0})",
                i + 1,
                i + 2,
                after_up / RPM_TO_RAD,
                DOWN_SHIFT / RPM_TO_RAD
            );
            assert!(after_up > DOWN_SHIFT * 1.1, "upshift {} lands in a downshift", i + 1);
            let after_down = DOWN_SHIFT * (GEARS[i] / GEARS[i + 1]);
            assert!(after_down < UP_SHIFT * 0.9, "downshift {} lands in an upshift", i + 2);
        }
    }

    /// The torque curve has to have a peak in the middle, or gear choice does
    /// not matter and the gearbox above is decoration.
    #[test]
    fn torque_curve_peaks_and_falls() {
        let mut best = (0usize, 0.0f32);
        for (i, t) in TORQUE.iter().enumerate() {
            if *t > best.1 {
                best = (i, *t);
            }
        }
        assert!(best.0 > 2 && best.0 < TORQUE.len() - 2, "peak torque is at an end of the range");
        assert!(
            *TORQUE.last().unwrap() < best.1 * 0.85,
            "engine does not fall off towards the limiter"
        );
        // Peak power, for the record.
        let (mut pw, mut at) = (0.0f32, 0.0f32);
        for (i, t) in TORQUE.iter().enumerate() {
            let w = i as f32 * TORQUE_STEP;
            if t * w > pw {
                pw = t * w;
                at = w / RPM_TO_RAD;
            }
        }
        println!("peak torque {:.0} N.m, peak power {:.0} kW at {at:.0} rpm", best.1, pw / 1000.0);
        assert!(pw > 200_000.0 && pw < 340_000.0, "peak power is {:.0} kW", pw / 1000.0);
    }

    /// Holding the throttle open with the wheels held still must rev the engine
    /// out rather than bogging it -- that is the clutch doing its job.
    #[test]
    fn engine_revs_against_a_stalled_car() {
        let mut d = Drivetrain::default();
        for _ in 0..480 {
            d.step(1.0, 0.0, 0.0, 0.0, 1.0 / 480.0);
        }
        let rpm = d.engine / RPM_TO_RAD;
        println!("one second at full throttle, wheels held: {rpm:.0} rpm");
        assert!(rpm > 2_000.0, "engine bogged to {rpm:.0} rpm instead of slipping the clutch");
        assert!(rpm <= REDLINE_RPM + 1.0, "engine passed the redline");

        // And the clutch is fully locked by the time the gearbox would ever
        // ask for drive, so slipping it never costs the car torque on track.
        assert!(cap(DOWN_SHIFT) > CLUTCH_CAP * 0.95, "clutch still slipping at the downshift point");
    }

    /// A car left in gear at idle must not creep away on its own.
    #[test]
    fn idle_in_gear_does_not_creep() {
        let mut d = Drivetrain::default();
        let out = d.step(0.0, 0.0, 0.0, 0.0, 1.0 / 480.0);
        println!("at idle, in first, off the throttle: {:.1} N.m to the axle", out.torque_l + out.torque_r);
        assert_eq!(cap(IDLE), 0.0, "the clutch is holding torque at idle");
        assert!(abs(out.torque_l + out.torque_r) < 1.0);
    }

    /// And it must never stall, however hard the driveline pulls on it.
    #[test]
    fn engine_cannot_be_stalled() {
        let mut d = Drivetrain::default();
        for _ in 0..2400 {
            d.step(0.0, 0.0, 0.0, 0.0, 1.0 / 480.0);
        }
        let rpm = d.engine / RPM_TO_RAD;
        println!("five seconds closed-throttle against a stopped axle: {rpm:.0} rpm");
        assert!(rpm > 500.0, "engine stalled at {rpm:.0} rpm");
    }

    /// A wheel spinning up has to lose torque to the one that is not.
    #[test]
    fn the_diff_sends_torque_to_the_slower_wheel() {
        let mut d = Drivetrain::default();
        // Left wheel spinning far faster than the right.
        let out = d.step(1.0, 90.0, 40.0, 20.0, 1.0 / 480.0);
        println!("torque split with 50 rad/s across the axle: l {:.0} r {:.0}", out.torque_l, out.torque_r);
        assert!(out.torque_r > out.torque_l, "diff fed the spinning wheel");
    }

    /// Lifting off has to give back torque, not just stop adding it.
    /// Reverse has to actually go backwards. The engine only turns one way, so
    /// a negative throttle means "press the pedal, in reverse gear" -- if that
    /// signed value reaches the torque curve unchanged it clamps to zero and
    /// the car sits there with the engine idling.
    #[test]
    fn reverse_drives_the_car_backwards() {
        let mut d = Drivetrain::default();
        // Stopped, asking for reverse.
        let mut out = Output::default();
        for _ in 0..480 {
            out = d.step(-0.7, 0.0, 0.0, 0.0, 1.0 / 480.0);
        }
        println!(
            "in reverse at {:.0} rpm: {:.0} N.m to the axle",
            d.engine / RPM_TO_RAD,
            out.torque_l + out.torque_r
        );
        assert_eq!(d.gear, -1.0, "never selected reverse");
        assert!(d.engine / RPM_TO_RAD > 1_200.0, "engine idled instead of revving");
        assert!(
            out.torque_l + out.torque_r < -200.0,
            "reverse made {:.0} N.m, which is not going anywhere",
            out.torque_l + out.torque_r
        );
    }

    #[test]
    fn closed_throttle_brakes_the_engine() {
        assert!(engine_torque(500.0, 0.0) < 0.0, "no engine braking at 4800 rpm");
        assert!(engine_torque(500.0, 1.0) > 0.0, "no drive at 4800 rpm");
    }

    /// Whatever speed a car is handed, the gear picked for it has to leave the
    /// engine somewhere it could actually be running.
    #[test]
    fn every_road_speed_has_a_sensible_gear() {
        let mut v = 0.0f32;
        while v <= 85.0 {
            let (gear, engine) = gear_for(v);
            let rpm = engine / RPM_TO_RAD;
            assert!(gear >= 1.0 && gear <= GEARS.len() as f32, "gear {gear} at {v} m/s");
            assert!(rpm >= IDLE_RPM - 1.0 && rpm <= REDLINE_RPM + 1.0, "{rpm:.0} rpm at {v} m/s");
            v += 1.0;
        }
        for v in [0.0f32, 15.0, 30.0, 50.0, 76.0] {
            let (gear, engine) = gear_for(v);
            println!("  {v:4.0} m/s -> gear {gear:.0} at {:.0} rpm", engine / RPM_TO_RAD);
        }
    }

    #[test]
    fn the_limiter_cuts_fuel() {
        let over = LIMITER + 10.0;
        assert!(engine_torque(over, 1.0) < 0.0, "full throttle past the limiter still drives");
    }
}
