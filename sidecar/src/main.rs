//! Authoritative physics sidecar for SpacetimeDB.
//!
//! An ordinary SpacetimeDB *client* that happens to be trusted: it subscribes to
//! player inputs, runs the simulation at a fixed 60 Hz, and writes the resulting
//! poses back through a reducer only it is allowed to call.
//!
//! Why not run the physics inside the module? Purely for headroom. A reducer is
//! a database transaction, and sixty of them a second per car, each doing
//! collision queries, is throughput the database could be spending on
//! everything else it is asked to do. Moving the work out here keeps the
//! *authority* -- clients still cannot assert their own position -- while
//! making the simulation a normal process you can profile, scale and restart
//! on its own.
//!
//!     cargo run -p sidecar --release -- --bots 5
//!
//! Options: --uri <url>  --db <name>  --bots <n>  --token <jwt>  --quiet

mod authority;
mod module_bindings;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use spacetimedb_sdk::{credentials, DbContext};

use authority::Authority;
use module_bindings::*;

const TICK: Duration = Duration::from_nanos(16_666_667);

struct Args {
    uri: String,
    db: String,
    bots: usize,
    token: Option<String>,
    quiet: bool,
}

fn parse_args() -> Args {
    let mut a = Args {
        uri: std::env::var("STDB_URI").unwrap_or_else(|_| "http://127.0.0.1:3000".into()),
        db: std::env::var("STDB_DB").unwrap_or_else(|_| "physics-sidecar".into()),
        bots: 5,
        token: std::env::var("STDB_TOKEN").ok(),
        quiet: false,
    };
    let mut argv = std::env::args().skip(1);
    while let Some(flag) = argv.next() {
        match flag.as_str() {
            "--uri" => a.uri = argv.next().unwrap_or_default(),
            "--db" => a.db = argv.next().unwrap_or_default(),
            "--bots" => a.bots = argv.next().and_then(|v| v.parse().ok()).unwrap_or(a.bots),
            "--token" => a.token = argv.next(),
            "--quiet" => a.quiet = true,
            other => eprintln!("ignoring unknown argument {other}"),
        }
    }
    a
}

fn creds() -> credentials::File {
    credentials::File::new("stdb-physics-sidecar")
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args();
    println!("physics sidecar -> {} / {}", args.uri, args.db);

    let subscribed = Arc::new(AtomicBool::new(false));
    let token = args.token.clone().or_else(|| creds().load().ok().flatten());

    let conn = DbConnection::builder()
        .with_uri(args.uri.as_str())
        .with_database_name(args.db.as_str())
        .with_token(token)
        .on_connect(|_ctx, id, tok| {
            if let Err(e) = creds().save(tok) {
                eprintln!("could not cache credentials: {e}");
            }
            println!("connected as {id}");
        })
        .on_connect_error(|_ctx, err| {
            eprintln!("connection failed: {err}");
            std::process::exit(1);
        })
        .on_disconnect(|_ctx, err| {
            match err {
                Some(e) => eprintln!("disconnected: {e}"),
                None => println!("disconnected"),
            }
            std::process::exit(1);
        })
        .build()?;

    // The sidecar needs the whole picture, so it subscribes to everything the
    // simulation touches. A player client subscribes to far less.
    conn.subscription_builder()
        .on_applied({
            let subscribed = subscribed.clone();
            move |_ctx| {
                subscribed.store(true, Ordering::SeqCst);
                println!("subscription applied");
            }
        })
        .on_error(|_ctx, err| {
            eprintln!("subscription error: {err}");
            if err.to_string().contains("input") {
                eprintln!("  `input` is private, so only the identity that published the module");
                eprintln!("  can read it. Start the sidecar with that identity's token:");
                eprintln!("    STDB_TOKEN=\"$(spacetime login show --token | awk '/auth token/ {{ print $NF }}')\"");
            }
            std::process::exit(1);
        })
        .subscribe([
            "SELECT * FROM config",
            "SELECT * FROM car",
            "SELECT * FROM car_state",
            "SELECT * FROM input",
            "SELECT * FROM player",
        ]);

    // Pump the connection until the initial state has landed.
    let deadline = Instant::now() + Duration::from_secs(20);
    while !subscribed.load(Ordering::SeqCst) {
        conn.frame_tick()?;
        if Instant::now() > deadline {
            return Err("timed out waiting for the initial subscription".into());
        }
        thread::sleep(Duration::from_millis(2));
    }

    check_config(&conn)?;

    // Claiming, resuming the tick clock and filling the grid with bots all
    // happen the moment this process is granted the authority -- which may be
    // now, or may be after the sidecar that currently holds it goes away. Until
    // then it stands by, following the race it is not running.
    let mut sim = Authority::new(conn.connection_id(), args.bots);
    println!(
        "simulating at {} Hz, publishing at {} Hz",
        physics::TICK_HZ,
        physics::TICK_HZ as u64 / authority::SNAPSHOT_EVERY
    );

    let mut pacer = Pacer::new();
    let mut next = Instant::now();
    loop {
        // --- wait for the tick boundary ----------------------------------
        let now = Instant::now();
        if next > now {
            pacer.wait(next);
        } else if now - next > TICK * 8 {
            // Fell a long way behind (a debugger, a laptop lid). Resync rather
            // than trying to catch up and stuttering everyone's prediction.
            eprintln!("[timing] {} ms behind, resyncing", (now - next).as_millis());
            next = now;
        }
        next += TICK;

        // Everything between here and the next boundary is the tick's real
        // cost: the SDK pump delivers the inputs and the echoes of our own
        // snapshots, and `step` does the rest. The wait above is not work.
        let work = Instant::now();
        conn.frame_tick()?;
        sim.step(&conn);
        sim.tick_time += work.elapsed();
        if !args.quiet {
            sim.report();
        }
    }
}

/// Fail loudly if the module was built against a different world size.
fn check_config(conn: &DbConnection) -> Result<(), Box<dyn std::error::Error>> {
    let cfg = conn
        .db
        .config()
        .id()
        .find(&0)
        .ok_or("module has no config row; publish it first")?;
    if cfg.max_cars as usize != physics::MAX_CARS {
        return Err(format!(
            "module allows {} car slots but this sidecar was built for {}",
            cfg.max_cars,
            physics::MAX_CARS
        )
        .into());
    }
    if cfg.tick_hz != physics::TICK_HZ {
        return Err(format!(
            "module expects {} Hz, physics crate runs at {} Hz",
            cfg.tick_hz,
            physics::TICK_HZ
        )
        .into());
    }
    // Not fatal, and not the sidecar's problem to fix -- it is the authority,
    // so whatever it computes is by definition correct. Worth saying out loud
    // though: it means the clients now have a fingerprint to disagree with, and
    // any of them still serving the old `physics.wasm` is about to.
    let mine = physics::fingerprint();
    if cfg.physics_fingerprint != 0 && cfg.physics_fingerprint != mine {
        println!(
            "physics fingerprint {mine:#010x} replaces {:#010x}; clients on the old \
             physics.wasm will mispredict until they are redeployed too",
            cfg.physics_fingerprint
        );
    }
    Ok(())
}

/// Sleeps to a deadline as accurately as this machine allows, and no longer.
///
/// A thread asked to sleep for `d` wakes some time *after* `d`, and how long
/// after is the platform's business: tens of microseconds on Linux, most of a
/// millisecond on Windows, a whole scheduling quantum on a busy machine. The
/// way to hit a 60 Hz boundary anyway is to sleep short and spin the rest --
/// but a fixed margin is a guess, and a guess is either a stutter or a core
/// burnt on every machine it was not tuned on.
///
/// So measure it instead. `margin` rises to the worst overshoot it has seen and
/// decays back down as wakeups improve, which spins for exactly as long as the
/// scheduler underneath currently needs and no longer. On the Windows box these
/// numbers come from it settles near 0.5 ms and spins 1.5 % of a core; a fixed
/// 1.5 ms, which is what covers the same machine on a bad day, spins 7 %. The
/// price is a tick that occasionally lands a tenth of a millisecond late, which
/// nothing downstream can tell from one that did not: the tick a snapshot
/// carries is a number, not a timestamp.
struct Pacer {
    margin: Duration,
}

impl Pacer {
    /// Where the estimate starts, and the range it may wander in. The floor is
    /// not zero because no wakeup is free; the ceiling is a quarter of a tick,
    /// past which spinning costs more than the accuracy is worth.
    const START: Duration = Duration::from_micros(1000);
    const FLOOR: Duration = Duration::from_micros(50);
    const CEILING: Duration = Duration::from_micros(4000);
    /// Shrink per wakeup. One percent at 60 Hz is a half-life of about a
    /// second: long enough that a bad wakeup still covers the next few dozen,
    /// short enough that one outlier does not tax the next ten seconds. Being
    /// wrong costs a tick that lands a few hundred microseconds late, which
    /// nothing downstream can tell from one that did not.
    const DECAY: f32 = 0.99;

    fn new() -> Self {
        Pacer { margin: Self::START }
    }

    fn wait(&mut self, deadline: Instant) {
        let remain = deadline.saturating_duration_since(Instant::now());
        if remain > self.margin {
            let ask = remain - self.margin;
            let before = Instant::now();
            thread::sleep(ask);
            let over = before.elapsed().saturating_sub(ask);
            self.margin = if over > self.margin {
                over.min(Self::CEILING)
            } else {
                self.margin.mul_f32(Self::DECAY).max(Self::FLOOR)
            };
        }
        while Instant::now() < deadline {
            std::hint::spin_loop();
        }
    }
}
