//! How battered a field of bots gets over a race, and how much of it they
//! spend not racing.
//!
//! `cargo run -p physics --example wear --release`
//!
//! A single race is chaotic: two cars that touch on lap one put the whole field
//! somewhere else by lap three, so one number out of one race says very little
//! about a change. This runs several fields with different drivers and reports
//! the aggregate, which is stable enough to tune against.

use physics::bot::BotBrain;
use physics::world::World;

const CARS: usize = 8;
const SECONDS: u64 = 150;
/// Speed below which a car is not racing, m/s.
const CRAWL: f32 = 5.0;

struct Race {
    /// Ticks spent below [`CRAWL`], summed over the field.
    stuck: u32,
    /// Ticks spent in contact with a barrier, summed over the field.
    wall: u32,
    laps: u32,
    best: f32,
    worst_lap: f32,
    severity: f32,
}

fn race(field: u32, damage_on: bool) -> Race {
    let mut w = World::new();
    let brains: Vec<BotBrain> = (0..CARS as u32).map(|i| BotBrain::new(field * 97 + i)).collect();
    for i in 0..CARS {
        w.spawn(i, i);
    }

    let mut r = Race { stuck: 0, wall: 0, laps: 0, best: 999.0, worst_lap: 0.0, severity: 0.0 };
    for tick in 0..60 * SECONDS {
        for i in 0..CARS {
            w.inputs[i] = brains[i].drive(i, &w.cars, w.active, &w.track, tick);
        }
        w.step((1 << CARS) - 1);
        for i in 0..CARS {
            if !damage_on {
                w.cars[i].set_damage(&Default::default());
            }
            if tick > 180 && w.cars[i].speed() < CRAWL {
                r.stuck += 1;
            }
            if w.cars[i].wall > 0.5 {
                r.wall += 1;
            }
        }
    }
    for i in 0..CARS {
        let c = &w.cars[i];
        r.laps += c.lap as u32;
        if c.best_lap > 0.0 && c.best_lap < r.best {
            r.best = c.best_lap;
        }
        if c.best_lap > r.worst_lap {
            r.worst_lap = c.best_lap;
        }
        r.severity += c.damage().severity() / CARS as f32;
    }
    r
}

fn main() {
    for damage_on in [true, false] {
        println!("--- damage {} ---", if damage_on { "on" } else { "suppressed" });
        println!("  field   laps   best    slowest   crawling   in contact   mean damage");
        let (mut stuck, mut wall) = (0u32, 0u32);
        for field in 0..5u32 {
            let r = race(field, damage_on);
            stuck += r.stuck;
            wall += r.wall;
            println!(
                "  {field:5}   {:4}   {:5.1}   {:7.1}   {:7.1} s   {:8.1} s   {:.2}",
                r.laps,
                r.best,
                r.worst_lap,
                r.stuck as f32 / 60.0,
                r.wall as f32 / 60.0,
                r.severity,
            );
        }
        let total = 5.0 * CARS as f32 * SECONDS as f32;
        println!(
            "  TOTAL crawling {:.1} s of {:.0} s of racing ({:.2} %), in contact {:.1} s",
            stuck as f32 / 60.0,
            total,
            stuck as f32 / 60.0 / total * 100.0,
            wall as f32 / 60.0,
        );
    }
}
