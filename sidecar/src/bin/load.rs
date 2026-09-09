//! Synthetic players, for measuring what a *client* costs the database.
//!
//!     cargo run -p sidecar --release --bin load -- --players 12
//!
//! The simulation is free to the database no matter how big the grid gets:
//! every car rides in one `push_states` call, so twenty-four cars and one car
//! are the same twenty transactions a second. Players are the term that
//! actually grows -- each one writes `set_input` at 30 Hz and is subscribed to
//! the snapshots coming back -- and that is the axis a bot can never measure,
//! because a bot has no connection and sends nothing.
//!
//! Every simulated player is its own connection and therefore its own
//! identity. That is the whole reason this exists rather than a handful of
//! browser tabs: the module keys `player` and `input` by identity, and tabs
//! share one stored token, so they would collide on a single row and measure
//! nothing at all.
//!
//! Options: --uri <url>  --db <name>  --players <n>  --hz <rate>  --seconds <n>

#[path = "../module_bindings/mod.rs"]
mod module_bindings;

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use spacetimedb_sdk::{DbContext, Table};

use module_bindings::*;

/// What the browser subscribes to, so the fan-out this measures is the fan-out
/// a real client actually causes.
const SUBSCRIPTION: [&str; 5] = [
    "SELECT * FROM config",
    "SELECT * FROM car",
    "SELECT * FROM car_state",
    "SELECT * FROM player",
    "SELECT * FROM lap_record",
];

struct Args {
    uri: String,
    db: String,
    players: usize,
    hz: f32,
    seconds: u64,
}

fn parse_args() -> Args {
    let mut a = Args {
        uri: std::env::var("STDB_URI").unwrap_or_else(|_| "http://127.0.0.1:3000".into()),
        db: std::env::var("STDB_DB").unwrap_or_else(|_| "physics-sidecar".into()),
        players: 8,
        hz: 30.0,
        seconds: 0,
    };
    let mut argv = std::env::args().skip(1);
    while let Some(flag) = argv.next() {
        match flag.as_str() {
            "--uri" => a.uri = argv.next().unwrap_or_default(),
            "--db" => a.db = argv.next().unwrap_or_default(),
            "--players" => a.players = argv.next().and_then(|v| v.parse().ok()).unwrap_or(a.players),
            "--hz" => a.hz = argv.next().and_then(|v| v.parse().ok()).unwrap_or(a.hz),
            "--seconds" => a.seconds = argv.next().and_then(|v| v.parse().ok()).unwrap_or(a.seconds),
            other => eprintln!("ignoring unknown argument {other}"),
        }
    }
    a
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args();
    println!(
        "load: {} players at {} Hz -> {} / {}",
        args.players, args.hz, args.uri, args.db
    );

    // No token on any of them: an unauthenticated connect is how you ask
    // SpacetimeDB for a brand new identity, which is exactly one per player.
    let subscribed = Arc::new(AtomicU32::new(0));
    let mut conns = Vec::with_capacity(args.players);
    for _ in 0..args.players {
        let conn = DbConnection::builder()
            .with_uri(args.uri.as_str())
            .with_database_name(args.db.as_str())
            .on_connect_error(|_ctx, err| {
                eprintln!("connection failed: {err}");
                std::process::exit(1);
            })
            .on_disconnect(|_ctx, err| {
                if let Some(e) = err {
                    eprintln!("disconnected: {e}");
                }
            })
            .build()?;
        conn.subscription_builder()
            .on_applied({
                let subscribed = subscribed.clone();
                move |_ctx| {
                    subscribed.fetch_add(1, Ordering::SeqCst);
                }
            })
            .on_error(|_ctx, err| eprintln!("subscription error: {err}"))
            .subscribe(SUBSCRIPTION);
        conns.push(conn);
    }

    let pump = |conns: &Vec<DbConnection>| {
        for c in conns {
            let _ = c.frame_tick();
        }
    };

    let deadline = Instant::now() + Duration::from_secs(30);
    while (subscribed.load(Ordering::SeqCst) as usize) < args.players {
        pump(&conns);
        if Instant::now() > deadline {
            return Err("timed out waiting for subscriptions".into());
        }
        thread::sleep(Duration::from_millis(2));
    }
    println!("{} connected and subscribed", args.players);

    for (i, c) in conns.iter().enumerate() {
        c.reducers.join_race(format!("LOAD{i:02}"), 0x5fa8ff)?;
    }

    // Joining is a round trip; let the grid settle before counting it.
    let settle = Instant::now() + Duration::from_millis(1500);
    while Instant::now() < settle {
        pump(&conns);
        thread::sleep(Duration::from_millis(5));
    }
    let seated = conns
        .iter()
        .filter(|c| {
            c.try_identity()
                .and_then(|id| c.db.player().identity().find(&id))
                .is_some_and(|p| p.car_id != 0)
        })
        .count();
    println!(
        "{seated} of {} took a grid slot ({} cars on track)",
        args.players,
        conns[0].db.car().count()
    );
    if seated < args.players {
        println!("(the rest were turned away: the grid holds 24 cars including bots)");
    }

    // --- the load itself -----------------------------------------------
    let period = Duration::from_secs_f32(1.0 / args.hz);
    let mut next = Instant::now();
    let mut seq: u32 = 0;
    let mut sent: u64 = 0;
    let mut failed: u64 = 0;
    let started = Instant::now();
    let mut last_report = Instant::now();

    loop {
        let now = Instant::now();
        if next > now {
            thread::sleep(next - now);
        }
        next += period;
        pump(&conns);

        // Stamp inputs where the authority actually is, so the sidecar's
        // scheduler treats them like a real client's rather than dropping a
        // backlog it can never promote.
        let tick = conns[0]
            .db
            .config()
            .id()
            .find(&0)
            .map_or(0, |c| c.server_tick + 5);
        seq += 1;
        let steer = ((seq % 120) as f32 - 60.0) / 60.0;
        for c in &conns {
            match c.reducers.set_input(seq, tick, 1.0, steer, 0.0, false) {
                Ok(()) => sent += 1,
                Err(_) => failed += 1,
            }
        }

        if last_report.elapsed() >= Duration::from_secs(1) {
            let secs = last_report.elapsed().as_secs_f32();
            println!(
                "[{:>4}s] {} players | {:>6.1} inputs/s sent | {failed} failed | {} cars",
                started.elapsed().as_secs(),
                seated,
                sent as f32 / secs,
                conns[0].db.car().count(),
            );
            sent = 0;
            last_report = Instant::now();
        }

        if args.seconds > 0 && started.elapsed() >= Duration::from_secs(args.seconds) {
            break;
        }
    }

    for c in &conns {
        let _ = c.reducers.leave_race();
    }
    let settle = Instant::now() + Duration::from_millis(500);
    while Instant::now() < settle {
        pump(&conns);
        thread::sleep(Duration::from_millis(5));
    }
    println!("done");
    Ok(())
}
