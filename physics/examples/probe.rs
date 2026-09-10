//! Scratch harness for reading numbers out of the vehicle model.
//!
//! `cargo run -p physics --example probe --release`

use physics::car::{self, CarInput, CarState};
use physics::{drivetrain, math, wheel};

fn tick(c: &mut CarState, inp: &CarInput) {
    for _ in 0..car::SUBSTEPS {
        car::integrate(c, inp, car::H);
    }
}

fn launch() {
    println!("--- standing start, full throttle ---");
    let mut c = CarState::default();
    c.place(math::V2::ZERO, 0.0);
    let inp = CarInput { throttle: 1.0, ..Default::default() };
    println!("  t      v km/h   gear   rpm   clutch  kappa_r   ax(g)  fz_r");
    for i in 0..=300 {
        if i % 15 == 0 {
            let u = c.forward_speed();
            let kappa = (c.w_rl * wheel::RADIUS - u) / math::max(u.abs(), 2.0);
            println!(
                "  {:.2}   {:6.1}   {:3}   {:5.0}   {:.2}   {:7.3}   {:5.2}",
                i as f32 * car::DT,
                u * 3.6,
                c.gear as i32,
                c.engine / drivetrain::RPM_TO_RAD,
                c.clutch,
                kappa,
                c.ax / car::G,
            );
        }
        tick(&mut c, &inp);
    }
}

/// Peak lateral g available at a speed, measured as a transient so that the
/// car does not have to hold the speed while saturated.
fn peak_lateral(v: f32) -> (f32, f32) {
    let mut best = (0.0f32, 0.0f32);
    let mut s = 0.05f32;
    while s <= 1.001 {
        let mut c = CarState::default();
        c.place(math::V2::ZERO, 0.0);
        c.vx = v;
        c.sync_drivetrain();
        let mut peak = 0.0f32;
        for _ in 0..90 {
            let inp = CarInput { throttle: 0.18, steer: s, ..Default::default() };
            tick(&mut c, &inp);
            let g = math::abs(c.ay) / car::G;
            if g > peak {
                peak = g;
            }
        }
        if peak > best.1 {
            best = (s, peak);
        }
        s += 0.05;
    }
    best
}

fn cornering() {
    println!("--- peak lateral g vs speed ---");
    for v in [15.0f32, 25.0, 35.0, 50.0, 65.0, 78.0] {
        let (lock, g) = peak_lateral(v);
        let modelled =
            car::grip_limit(v, &Default::default()) / (car::MASS * car::G);
        println!(
            "  {v:4.0} m/s: {g:.2} g at lock {lock:.2}   (grip_limit says {modelled:.2} g)"
        );
    }
}

fn balance() {
    println!("--- understeer balance: slip angles at the limit, 35 m/s ---");
    let mut c = CarState::default();
    c.place(math::V2::ZERO, 0.0);
    c.vx = 35.0;
    c.sync_drivetrain();
    for _ in 0..120 {
        let inp = CarInput { throttle: 0.2, steer: 1.0, ..Default::default() };
        tick(&mut c, &inp);
    }
    println!(
        "  slip f {:.1} deg, r {:.1} deg, roll {:.1} deg, ay {:.2} g",
        c.slip_f.to_degrees(),
        c.slip_r.to_degrees(),
        c.roll.to_degrees(),
        c.ay / car::G
    );
}

/// What a tick of the full world costs, which is the number the sidecar's
/// status line reports and the whole reason for running physics in a sidecar.
fn cost() {
    use std::time::Instant;
    println!("--- cost per tick ---");
    for n in [1usize, 7, 16, 24] {
        let mut w = physics::World::new();
        let brains: Vec<physics::bot::BotBrain> =
            (0..n as u32).map(physics::bot::BotBrain::new).collect();
        for i in 0..n {
            w.spawn(i, i);
        }
        let mask = if n >= 32 { u32::MAX } else { (1u32 << n) - 1 };
        // Warm up, and get the field spread out around the lap.
        for tick in 0..600u64 {
            for i in 0..n {
                w.inputs[i] = brains[i].drive(i, &w.cars, w.active, &w.track, tick);
            }
            w.step(mask);
        }
        let runs = 3000u64;
        // Both halves timed inside the same run: measuring them in separate
        // passes means the second one drives a field that has already piled
        // into the barriers, which is not the same work at all.
        let mut ai = std::time::Duration::ZERO;
        let mut sim = std::time::Duration::ZERO;
        for tick in 600..600 + runs {
            let t0 = Instant::now();
            for i in 0..n {
                w.inputs[i] = brains[i].drive(i, &w.cars, w.active, &w.track, tick);
            }
            ai += t0.elapsed();
            let t1 = Instant::now();
            w.step(mask);
            sim += t1.elapsed();
        }
        let ai = ai.as_secs_f64() * 1e6 / runs as f64;
        let sim = sim.as_secs_f64() * 1e6 / runs as f64;
        println!(
            "  {n:2} cars: {:6.1} us/tick  ({:.2}% of a 60 Hz budget)  = {sim:5.1} physics + {ai:4.1} bot AI",
            ai + sim,
            (ai + sim) / 16_666.0 * 100.0
        );
    }
}

fn main() {
    cost();
    launch();
    cornering();
    balance();
}
