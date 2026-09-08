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

use spacetimedb_sdk::{credentials, DbContext, Identity, Table};

use authority::Authority;
use module_bindings::*;

const TICK: Duration = Duration::from_nanos(16_666_667);
/// Stop sleeping this long before the deadline and spin instead. No mainstream
/// scheduler wakes a thread accurately enough for a 60 Hz tick on its own --
/// Windows is the worst of them, but Linux and macOS overshoot too.
const SPIN_MARGIN: Duration = Duration::from_micros(1500);

const BOT_NAMES: [&str; 12] = [
    "VECTOR", "NITRO", "HALCYON", "RIPTIDE", "ZEPHYR", "OBSIDIAN", "QUASAR", "MAVERICK", "TEMPEST",
    "CINDER", "ONYX", "VAPOR",
];
const BOT_COLORS: [u32; 12] = [
    0xff4d6d, 0x4dd2ff, 0xffd166, 0x8affc1, 0xc77dff, 0xff9f45, 0x5fa8ff, 0xff6ec7, 0x9dff5f,
    0x00e5c0, 0xffe066, 0xff5f5f,
];

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
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        let next = || argv.get(i + 1).cloned().unwrap_or_default();
        match argv[i].as_str() {
            "--uri" => {
                a.uri = next();
                i += 1;
            }
            "--db" => {
                a.db = next();
                i += 1;
            }
            "--bots" => {
                a.bots = next().parse().unwrap_or(5);
                i += 1;
            }
            "--token" => {
                a.token = Some(next());
                i += 1;
            }
            "--quiet" => a.quiet = true,
            other => eprintln!("ignoring unknown argument {other}"),
        }
        i += 1;
    }
    a
}

fn creds() -> credentials::File {
    credentials::File::new("stdb-physics-sidecar")
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args();
    println!("physics sidecar -> {} / {}", args.uri, args.db);

    let connected = Arc::new(AtomicBool::new(false));
    let subscribed = Arc::new(AtomicBool::new(false));
    let identity: Arc<std::sync::Mutex<Option<Identity>>> = Arc::default();

    let token = args.token.clone().or_else(|| creds().load().ok().flatten());

    let conn = DbConnection::builder()
        .with_uri(args.uri.as_str())
        .with_database_name(args.db.as_str())
        .with_token(token)
        .on_connect({
            let connected = connected.clone();
            let identity = identity.clone();
            move |_ctx, id, tok| {
                if let Err(e) = creds().save(tok) {
                    eprintln!("could not cache credentials: {e}");
                }
                *identity.lock().unwrap() = Some(id);
                connected.store(true, Ordering::SeqCst);
                println!("connected as {id}");
            }
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

    let me = identity.lock().unwrap().expect("identity after connect");
    check_config(&conn)?;

    conn.reducers.claim_authority()?;
    let mut sim = Authority::new(me);

    // Resume the tick counter where the last authority left off. The module
    // rejects snapshots older than the row it already holds, so a sidecar that
    // restarted from tick zero would have every write silently ignored.
    if let Some(cfg) = conn.db.config().id().find(&0) {
        sim.world.tick = cfg.server_tick + 1;
        println!("resuming simulation clock at tick {}", sim.world.tick);
    }

    // Give the claim a moment to land, then fill the grid with bots.
    let settle = Instant::now() + Duration::from_millis(750);
    while Instant::now() < settle {
        conn.frame_tick()?;
        thread::sleep(Duration::from_millis(5));
    }
    reconcile_bots(&conn, args.bots)?;

    println!(
        "simulating at {} Hz, publishing at {} Hz",
        physics::TICK_HZ,
        physics::TICK_HZ as u64 / authority::SNAPSHOT_EVERY
    );

    let mut next = Instant::now();
    loop {
        // --- wait for the tick boundary ----------------------------------
        let now = Instant::now();
        if next > now {
            let remain = next - now;
            if remain > SPIN_MARGIN {
                thread::sleep(remain - SPIN_MARGIN);
            }
            while Instant::now() < next {
                std::hint::spin_loop();
            }
        } else if now - next > TICK * 8 {
            // Fell a long way behind (a debugger, a laptop lid). Resync rather
            // than trying to catch up and stuttering everyone's prediction.
            eprintln!("[timing] {} ms behind, resyncing", (now - next).as_millis());
            next = now;
        }
        next += TICK;

        conn.frame_tick()?;
        sim.step(&conn);
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
    Ok(())
}

/// Bring the bot field to `count`, keeping whoever is already out there.
///
/// A restarting sidecar inherits the previous one's bots -- it resumes them from
/// their last published pose -- so this only adds or removes the difference.
fn reconcile_bots(conn: &DbConnection, count: usize) -> Result<(), Box<dyn std::error::Error>> {
    let want = count.min(BOT_NAMES.len());
    let existing: Vec<(u32, String)> = conn
        .db
        .car()
        .iter()
        .filter(|c| c.is_bot)
        .map(|c| (c.car_id, c.name.clone()))
        .collect();

    for (car_id, name) in existing.iter().skip(want) {
        println!("retiring bot {name}");
        conn.reducers.despawn_bot(*car_id)?;
    }

    let taken: Vec<&str> = existing.iter().map(|(_, n)| n.as_str()).collect();
    let mut added = 0;
    for (i, name) in BOT_NAMES.iter().enumerate() {
        if existing.len() + added >= want {
            break;
        }
        if taken.contains(name) {
            continue;
        }
        conn.reducers.spawn_bot((*name).to_string(), BOT_COLORS[i])?;
        added += 1;
    }
    println!(
        "bots: {} already racing, {added} added, {} retired",
        existing.len().min(want),
        existing.len().saturating_sub(want)
    );
    Ok(())
}
