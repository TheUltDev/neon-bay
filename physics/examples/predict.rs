//! How well a rival's pose can be guessed between snapshots.
//!
//!     cargo run -p physics --example predict --release
//!
//! The browser gets twenty snapshots a second and has to place the other
//! twenty-three cars at the tick it is *itself* on, which is a round trip
//! further along. This measures the error in that guess, for four ways of
//! making it:
//!
//! * **straight line** -- carry the pose forward along its velocity;
//! * **constant turn rate** -- rotate the velocity as it goes, so a car
//!   mid-corner follows the arc instead of flying off the tangent;
//! * **held input** -- run the real physics forward from the snapshot, on the
//!   controller state the authority published with it, held for the whole lead;
//! * **zero input** -- the same physics with the pedals released. The control:
//!   it separates having real dynamics from having the real pedals, and only
//!   the gap between it and *held input* is an argument for putting a rival's
//!   throttle on the wire.
//!
//! The first two are closed form and take nothing but the pose. The last two
//! cost a physics step per rival per tick, which is the whole grid instead of
//! one car, so they have to earn it.
//!
//! The arrangement is the client's, not an abstraction of it. One car is
//! *local* -- stepped on its own real inputs, because a client knows what it
//! did with its own controls -- and every other car on the grid is scored as a
//! rival. Under the two closed-form schemes the rivals are parked at their
//! extrapolated pose and never integrated, exactly as `placeRemotes` used to
//! leave them; under the two physics schemes the whole grid is stepped
//! together, so a rival that gets hit during the lead reacts to it.
//!
//! Bots are a hard test for held input and an easy one for the extrapolations.
//! A pure-pursuit controller re-decides its steering sixty times a second and
//! wobbles about the line; a human holding a key does not. Whatever margin
//! held input wins here, it wins by more against a real driver.

use physics::bot::BotBrain;
use physics::car::{CarInput, CarState};
use physics::math::{abs, cos, sin, wrap_pi, V2};
use physics::{World, DT};

/// Ticks between snapshots. `SNAPSHOT_EVERY` in the sidecar: 60 Hz / 3 = 20 Hz.
const SNAPSHOT_EVERY: u64 = 3;
/// Leads to score, in ticks, ascending. A real round trip needs the middle of
/// this range; the far end is here to find where holding an input stops paying.
const LEADS: [u64; 5] = [3, 6, 12, 18, 24];
/// `MAX_LEAD` in `web/src/net.ts`: the clamp the extrapolations run under, so
/// they are scored as the client actually used them.
const MAX_LEAD: f32 = 0.3;

const BOTS: usize = 8;
const MASK: u32 = (1 << BOTS) - 1;
const WARMUP: u64 = 600;
const TICKS: u64 = 60 * 180;

const LINE: usize = 0;
const TURN: usize = 1;
const HELD: usize = 2;
const ZERO: usize = 3;
const SCHEMES: [&str; 4] = ["straight line", "constant turn rate", "held input", "zero input"];

/// The three populations the errors are reported over.
const ALL: usize = 0;
/// A rival is *hard* when it is doing something a straight line cannot follow:
/// braking, or turning fast enough to be in a corner rather than correcting.
/// The means over everything are dominated by the cars doing neither.
const HARD: usize = 1;
/// A rival is *struck* when it takes a hit from another car during the lead --
/// after the snapshot being carried forward was taken, so the impulse is not in
/// it. This used to be the case no scheme could follow, and it is the one to
/// watch: an extrapolated pose cannot know about a collision that has not
/// happened yet, but a whole grid stepped together can, because the car that
/// does the hitting is in the same world doing the same physics.
const STRUCK: usize = 2;
const SLICES: usize = 3;

fn is_hard(c: &CarState, inp: &CarInput) -> bool {
    inp.brake > 0.1 || abs(c.omega) > 0.35
}

/// Did car `i` take a hit from another car between `t0` and `t0 + lead`?
///
/// `impact` counts barrier contact too and `wall` is raised for the tick a car
/// is scraping one, so requiring `wall` to be clear keeps this to panel on
/// panel. A tick that manages both at once is missed, which makes this an
/// undercount rather than an overcount.
fn was_struck(truth: &[[CarState; BOTS]], t0: usize, lead: usize, i: usize) -> bool {
    (t0 + 1..=t0 + lead).any(|t| truth[t][i].impact > 1.0 && truth[t][i].wall < 0.5)
}

#[derive(Clone, Copy, Default)]
struct Pose {
    p: V2,
    heading: f32,
}

impl Pose {
    fn of(c: &CarState) -> Pose {
        Pose { p: c.pos(), heading: c.heading }
    }
}

/// The snapshot as it actually reaches the browser.
///
/// `slip_f`, `slip_r`, `impact` and `wall` are the four fields of the record
/// that are not on the wire. Each is written before it is next read, so
/// dropping them is exact rather than approximate -- and if that ever stops
/// being true, a physics scheme run from a wire record is where it shows up.
fn wire(c: &CarState) -> CarState {
    CarState { slip_f: 0.0, slip_r: 0.0, impact: 0.0, wall: 0.0, ..*c }
}

/// Carry a pose forward by `dt` seconds. With `turn`, the velocity is rotated
/// as it goes and the heading turns with it; without, both are held fixed.
fn extrapolate(c: &CarState, dt: f32, turn: bool) -> Pose {
    let dt = if dt > MAX_LEAD { MAX_LEAD } else { dt };
    if !turn {
        return Pose { p: V2::new(c.x + c.vx * dt, c.y + c.vy * dt), heading: c.heading };
    }
    let w = c.omega;
    let th = w * dt;
    let straight = abs(w) < 1e-3;
    let s = if straight { dt } else { sin(th) / w };
    let k = if straight { 0.0 } else { (1.0 - cos(th)) / w };
    Pose {
        p: V2::new(c.x + c.vx * s - c.vy * k, c.y + c.vx * k + c.vy * s),
        heading: c.heading + th,
    }
}

/// Every error one scheme made at one lead, kept whole so the tail can be
/// asked for rather than estimated, and split by which populations the sample
/// belongs to. Every sample is in [`ALL`]; the other two overlap freely.
#[derive(Default)]
struct Tally {
    pos: [Vec<f32>; SLICES],
    head: [Vec<f32>; SLICES],
}

impl Tally {
    fn push(&mut self, truth: &CarState, guess: &Pose, into: &[usize]) {
        let p = truth.pos().sub(guess.p).len();
        let h = abs(wrap_pi(truth.heading - guess.heading));
        for s in into {
            self.pos[*s].push(p);
            self.head[*s].push(h);
        }
    }
}

/// Which populations one sample belongs to.
fn slices(hard: bool, struck: bool) -> Vec<usize> {
    let mut v = vec![ALL];
    if hard {
        v.push(HARD);
    }
    if struck {
        v.push(STRUCK);
    }
    v
}

/// Mean, 90th, 99th and worst, in that order. Sorts in place.
fn stats(v: &mut [f32]) -> (f32, f32, f32, f32) {
    if v.is_empty() {
        return (0.0, 0.0, 0.0, 0.0);
    }
    v.sort_by(f32::total_cmp);
    let mean = v.iter().map(|x| *x as f64).sum::<f64>() / v.len() as f64;
    let at = |q: f64| v[(((v.len() - 1) as f64) * q) as usize];
    (mean as f32, at(0.90), at(0.99), v[v.len() - 1])
}

fn main() {
    // ---- the race that is being predicted --------------------------------
    let mut w = World::new();
    let brains: Vec<BotBrain> = (0..BOTS as u32).map(BotBrain::new).collect();
    for i in 0..BOTS {
        w.spawn(i, i);
    }

    let end = (WARMUP + TICKS) as usize;
    // `truth[t]` is the grid at tick `t`; `applied[t]` is the controller state
    // that produced it, which is the pair the sidecar publishes together.
    let mut truth: Vec<[CarState; BOTS]> = Vec::with_capacity(end + 1);
    let mut applied: Vec<[CarInput; BOTS]> = Vec::with_capacity(end + 1);
    truth.push(core::array::from_fn(|i| w.cars[i]));
    applied.push([CarInput::default(); BOTS]);
    for _ in 0..end {
        let tick = w.tick;
        for i in 0..BOTS {
            w.inputs[i] = brains[i].drive(i, &w.cars, w.active, &w.track, tick);
        }
        let inp: [CarInput; BOTS] = core::array::from_fn(|i| w.inputs[i]);
        w.step(MASK);
        truth.push(core::array::from_fn(|i| w.cars[i]));
        applied.push(inp);
    }

    // ---- what each scheme would have guessed ------------------------------
    let mut tally: [[Tally; 4]; LEADS.len()] =
        core::array::from_fn(|_| core::array::from_fn(|_| Tally::default()));
    let far = *LEADS.iter().max().unwrap() as usize;
    // One world, rebuilt from each snapshot rather than allocated per run: a
    // `World` carries the whole circuit, and 3600 of them is 3600 track builds.
    let mut pw = World::new();
    let mut snapshots = 0u64;

    let mut t0 = WARMUP as usize;
    while t0 + far <= end {
        // Which car is holding the controller. Rotated, so no single bot's
        // driving decides what the table says.
        let local = (t0 / SNAPSHOT_EVERY as usize) % BOTS;

        for (li, lead) in LEADS.iter().enumerate() {
            let dt = *lead as f32 * DT;
            for i in (0..BOTS).filter(|i| *i != local) {
                let snap = wire(&truth[t0][i]);
                let into = slices(
                    is_hard(&snap, &applied[t0][i]),
                    was_struck(&truth, t0, *lead as usize, i),
                );
                let real = &truth[t0 + *lead as usize][i];
                tally[li][LINE].push(real, &extrapolate(&snap, dt, false), &into);
                tally[li][TURN].push(real, &extrapolate(&snap, dt, true), &into);
            }
        }

        for scheme in [HELD, ZERO] {
            for i in 0..BOTS {
                pw.cars[i] = wire(&truth[t0][i]);
                pw.inputs[i] = if scheme == HELD { applied[t0][i] } else { CarInput::default() };
            }
            pw.active = MASK;
            pw.tick = t0 as u64;
            let mut li = 0;
            for l in 1..=far {
                // The one car whose future the client does know.
                pw.inputs[local] = applied[t0 + l][local];
                pw.step(MASK);
                if LEADS[li] as usize != l {
                    continue;
                }
                for i in (0..BOTS).filter(|i| *i != local) {
                    let into =
                        slices(is_hard(&truth[t0][i], &applied[t0][i]), was_struck(&truth, t0, l, i));
                    tally[li][scheme].push(&truth[t0 + l][i], &Pose::of(&pw.cars[i]), &into);
                }
                li += 1;
            }
        }

        snapshots += 1;
        t0 += SNAPSHOT_EVERY as usize;
    }

    // ---- what it all came to ----------------------------------------------
    let rivals = snapshots * (BOTS - 1) as u64;
    println!(
        "{BOTS} bots, {} s of racing, {snapshots} snapshots, {rivals} rival poses scored per lead\n",
        TICKS / 60,
    );
    report("every rival", &mut tally, ALL);
    println!(
        "\nThe same runs, counting only rivals that were braking or turning at more\nthan 0.35 rad/s when the snapshot was taken. This is the case the netcode is\nfor: the mean above is mostly cars going in a straight line, where the four\nschemes agree and none of this matters."
    );
    report("braking or cornering", &mut tally, HARD);
    println!(
        "\nAnd only rivals that were hit by another car during the lead -- after the\nsnapshot being carried forward was taken, so the impulse is not in it. The\ntwo extrapolations cannot follow a collision that has not happened yet. A\nwhole grid stepped together can, because whoever does the hitting is in the\nsame world running the same physics."
    );
    report("struck by another car", &mut tally, STRUCK);
}

fn report(title: &str, tally: &mut [[Tally; 4]; LEADS.len()], slice: usize) {
    println!("\n  {title}");
    println!("    lead / scheme               mean      p90       p99     worst    heading p99   n");
    for (li, lead) in LEADS.iter().enumerate() {
        println!("    {:>2} ticks ({:>3} ms)", lead, lead * 1000 / 60);
        for (s, name) in SCHEMES.iter().enumerate() {
            let t = &mut tally[li][s];
            if t.pos[slice].is_empty() {
                continue;
            }
            let n = t.pos[slice].len();
            let (mean, p90, p99, worst) = stats(&mut t.pos[slice]);
            let (_, _, h99, _) = stats(&mut t.head[slice]);
            println!(
                "      {name:<22} {mean:6.3} m  {p90:6.3} m  {p99:6.3} m  {worst:6.2} m   {:5.2} deg  {n:>6}",
                h99.to_degrees()
            );
        }
    }
}
