//! Prints a bit-exact fingerprint of a scripted 1200-tick run.
//!
//! `scripts/verify-determinism.mjs` runs the same script through the wasm build
//! and diffs the output. If those two ever disagree, client prediction is
//! guessing rather than reproducing, and every corner would need a correction.
//!
//!     cargo run -p physics --release --example trace

use physics::{CarInput, World};

/// Inputs are derived from integer arithmetic only, so the *driver* is
/// identical on both sides and the only thing under test is the simulation.
fn scripted(tick: u64) -> CarInput {
    CarInput {
        throttle: if (tick / 37) % 3 == 0 { 0.0 } else { 1.0 },
        steer: ((tick % 240) as f32 - 120.0) / 120.0,
        brake: if (tick / 53) % 5 == 0 { 0.5 } else { 0.0 },
        handbrake: if (tick / 97) % 7 == 0 { 1.0 } else { 0.0 },
    }
}

fn main() {
    // First row is the fingerprint the sidecar publishes and the browser checks
    // itself against at runtime. Diffing it here as well means the build-time
    // check covers the mechanism the deployed system relies on, not just the
    // arithmetic underneath it.
    println!("fp   {:08x}", physics::fingerprint());

    let mut w = World::new();
    w.spawn(0, 0);
    w.spawn(1, 3);
    for tick in 0..1200u64 {
        w.inputs[0] = scripted(tick);
        w.inputs[1] = scripted(tick + 511);
        w.step(0b11);
        if tick % 300 == 299 {
            print_row(tick, &w);
        }
    }
    print_row(1199, &w);
}

fn print_row(tick: u64, w: &World) {
    for i in 0..2 {
        let c = w.cars[i];
        println!(
            "t{:04} car{} {:08x} {:08x} {:08x} {:08x} {:08x} {:08x}",
            tick,
            i,
            c.x.to_bits(),
            c.y.to_bits(),
            c.heading.to_bits(),
            c.vx.to_bits(),
            c.vy.to_bits(),
            c.omega.to_bits()
        );
    }
}
