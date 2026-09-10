//! How well a rival's pose can be guessed between snapshots.
//!
//!     cargo run -p physics --example predict --release
//!
//! The browser never simulates anybody else's car. It gets 20 snapshots a
//! second and has to draw the other twenty-three cars at the tick it is
//! *itself* on, which is a round trip further along. This measures the error in
//! that guess, for the two ways of making it:
//!
//! * **straight line** carry the pose forward along its velocity;
//! * **constant turn rate** rotate the velocity as it goes, so a car mid-corner
//!   follows the arc instead of flying off the tangent.
//!
//! Both are exactly what `web/src/net.ts` does in `predict()` -- including its
//! clamp on how far ahead it will extrapolate -- run against the authority's
//! own snapshot cadence and scored against where the car really was.

use physics::bot::BotBrain;
use physics::car::CarState;
use physics::math::{cos, sin, V2};
use physics::World;

/// Ticks between snapshots. `SNAPSHOT_EVERY` in the sidecar: 60 Hz / 3 = 20 Hz.
const SNAPSHOT_EVERY: u64 = 3;
/// Leads to score, in ticks.
const LEADS: [u64; 3] = [3, 6, 12];
/// `MAX_LEAD` in `web/src/net.ts`: the client will not extrapolate further
/// ahead than this many seconds however far behind the snapshot is.
const MAX_LEAD: f32 = 0.3;

const BOTS: usize = 6;
const WARMUP: u64 = 600;
const TICKS: u64 = 60 * 120;

/// Carry a pose forward by `dt` seconds. With `turn`, the velocity is rotated
/// as it goes; without, it is held fixed.
fn extrapolate(c: &CarState, dt: f32, turn: bool) -> V2 {
    let dt = if dt > MAX_LEAD { MAX_LEAD } else { dt };
    if !turn {
        return V2::new(c.x + c.vx * dt, c.y + c.vy * dt);
    }
    let w = c.omega;
    let th = w * dt;
    let straight = physics::math::abs(w) < 1e-3;
    let s = if straight { dt } else { sin(th) / w };
    let k = if straight { 0.0 } else { (1.0 - cos(th)) / w };
    V2::new(c.x + c.vx * s - c.vy * k, c.y + c.vx * k + c.vy * s)
}

fn main() {
    let mut w = World::new();
    let brains: Vec<BotBrain> = (0..BOTS as u32).map(BotBrain::new).collect();
    for i in 0..BOTS {
        w.spawn(i, i);
    }

    // Newest snapshot per car, and how long ago it was taken.
    let mut snap = [CarState::default(); BOTS];
    let mut snap_tick = [0u64; BOTS];
    // Squared error would flatter the small leads; these are plain means.
    let mut sum = [[0.0f64; 2]; LEADS.len()];
    let mut worst = [[0.0f32; 2]; LEADS.len()];
    let mut n = 0u64;
    // A car's true pose, `lead` ticks after each snapshot, has to be looked up
    // when the simulation gets there -- so the guesses are parked until then.
    let mut pending: Vec<(u64, usize, usize, V2, V2)> = Vec::new();

    for tick in 0..WARMUP + TICKS {
        for i in 0..BOTS {
            w.inputs[i] = brains[i].drive(i, &w.cars, w.active, &w.track, tick);
        }
        w.step((1 << BOTS) - 1);

        // Score any guess whose target tick is now.
        pending.retain(|&(at, i, li, straight, turn)| {
            if at != w.tick {
                return true;
            }
            let truth = w.cars[i].pos();
            let e_s = truth.sub(straight).len();
            let e_t = truth.sub(turn).len();
            sum[li][0] += e_s as f64;
            sum[li][1] += e_t as f64;
            if e_s > worst[li][0] {
                worst[li][0] = e_s;
            }
            if e_t > worst[li][1] {
                worst[li][1] = e_t;
            }
            false
        });

        if w.tick % SNAPSHOT_EVERY != 0 {
            continue;
        }
        for i in 0..BOTS {
            snap[i] = w.cars[i];
            snap_tick[i] = w.tick;
        }
        if tick < WARMUP {
            continue;
        }
        n += 1;
        for (li, lead) in LEADS.iter().enumerate() {
            for i in 0..BOTS {
                let dt = *lead as f32 / 60.0;
                pending.push((
                    snap_tick[i] + lead,
                    i,
                    li,
                    extrapolate(&snap[i], dt, false),
                    extrapolate(&snap[i], dt, true),
                ));
            }
        }
    }

    let scored = (n * BOTS as u64) as f64;
    println!(
        "{BOTS} bots, {} s of racing, {} snapshots scored per lead\n",
        TICKS / 60,
        n * BOTS as u64
    );
    println!("  lead              straight line     constant turn rate    gain");
    for (li, lead) in LEADS.iter().enumerate() {
        let s = sum[li][0] / scored;
        let t = sum[li][1] / scored;
        println!(
            "  {lead:2} ticks ({:3} ms)      {s:.3} m              {t:.3} m         {:.0}%",
            lead * 1000 / 60,
            (1.0 - t / s) * 100.0
        );
    }
    println!("\n  worst case, same runs:");
    for (li, lead) in LEADS.iter().enumerate() {
        println!(
            "  {lead:2} ticks              {:.3} m              {:.3} m",
            worst[li][0], worst[li][1]
        );
    }
}
