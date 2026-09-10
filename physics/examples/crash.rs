//! What actually happens when two cars touch.
//!
//! `cargo run -p physics --example crash --release`
//!
//! Three questions, and they are different questions:
//!
//! 1. **Is the hit itself right?** A car-to-car impact is mostly plastic. Sheet
//!    metal that bends does not give the energy back, so the separation speed
//!    should be a small fraction of the closing speed and should get smaller as
//!    the closing speed rises.
//! 2. **Does the client predict the same hit the authority resolves?** What the
//!    browser assumes about the car it just hit -- its mass, and whether the
//!    contact is allowed to move it -- decides whether its prediction is nearly
//!    right or completely wrong, and the driver feels the difference at once.
//! 3. **Does the contact resolve before it is deep?** Penetration that is
//!    allowed to get large picks the wrong separating axis and is then pushed
//!    out along it.

use physics::car::{CarInput, CarState};
use physics::math::V2;
use physics::world::World;

/// Set up two cars on the start straight, `gap` metres apart along it, and give
/// the chaser `closing` m/s of it.
fn stage(gap: f32, closing: f32, lead_speed: f32) -> (World, V2) {
    let mut w = World::new();
    w.spawn(0, 0);
    w.spawn(1, 1);

    // Line them both up on the same heading, on the centreline, so the geometry
    // of the hit is not confounded by the grid's stagger.
    let (p, heading) = w.track.grid_slot(0);
    let fwd = V2::from_angle(heading);

    for (i, along) in [(0usize, 0.0f32), (1usize, gap)] {
        let at = p.add(fwd.scale(along));
        w.cars[i].place(at, heading);
        w.cars[i].active = 1.0;
    }
    let speed = [closing + lead_speed, lead_speed];
    for i in 0..2 {
        w.cars[i].vx = fwd.x * speed[i];
        w.cars[i].vy = fwd.y * speed[i];
        w.cars[i].sync_drivetrain();
        let hit = w.track.nearest(w.cars[i].pos(), physics::track::NO_HINT);
        w.cars[i].seg = hit.idx as f32;
        w.cars[i].s = hit.s;
        w.cars[i].lat = hit.lat;
    }
    (w, fwd)
}

/// Forward speed of car `i` along `fwd`.
fn along(w: &World, i: usize, fwd: V2) -> f32 {
    w.cars[i].vel().dot(fwd)
}

fn kinetic(c: &CarState) -> f32 {
    0.5 * physics::car::MASS * c.vel().len_sq() + 0.5 * physics::car::IZ * c.omega * c.omega
}

/// Run the pair together through the contact and stop as soon as it is over.
///
/// Measured across the hit rather than after a second and a half of coasting:
/// drag and rolling resistance take more energy out of two cars in a second
/// than a moderate shunt does, and they are not what is being asked about.
///
/// `mask` is what gets integrated: `0b11` is both cars, which is what the
/// sidecar and the browser now both pass, and `0b01` is one car with the other
/// as scenery, which is what the browser used to.
fn run(gap: f32, closing: f32, lead_speed: f32, mask: u32) -> Report {
    let (mut w, fwd) = stage(gap, closing, lead_speed);
    let coast = CarInput::default();
    w.inputs[0] = coast;
    w.inputs[1] = coast;

    let mut before = 0.0;
    let mut peak = 0.0f32;
    let mut touching = false;
    let mut clear = 0;

    for _ in 0..240 {
        if !touching {
            before = kinetic(&w.cars[0]) + kinetic(&w.cars[1]);
        }
        w.step(mask);
        let imp = w.cars[0].impact + w.cars[1].impact;
        if imp > 1.0 {
            touching = true;
            clear = 0;
            if imp > peak {
                peak = imp;
            }
        } else if touching {
            clear += 1;
            if clear > 4 {
                break;
            }
        }
    }

    let after = kinetic(&w.cars[0]) + kinetic(&w.cars[1]);
    let _ = peak;
    Report {
        chaser: along(&w, 0, fwd),
        lead: along(&w, 1, fwd),
        absorbed: (before - after) / 1000.0,
        crush: w.cars[0].damage(),
    }
}

struct Report {
    chaser: f32,
    lead: f32,
    absorbed: f32,
    crush: physics::damage::Damage,
}

fn rear_end() {
    println!("--- rear-ended: chaser into a car doing 20 m/s, both coasting ---");
    println!("  closing   chaser   lead   separation   of closing   absorbed   chaser's nose");
    for closing in [2.0f32, 6.0, 12.0, 20.0, 30.0] {
        let r = run(6.0, closing, 20.0, 0b11);
        let sep = r.lead - r.chaser;
        println!(
            "  {closing:5.0} m/s   {:5.1}   {:5.1}   {sep:7.2} m/s   {:7.0} %   {:6.1} kJ   {:.3} m",
            r.chaser,
            r.lead,
            sep / closing * 100.0,
            r.absorbed,
            r.crush.front,
        );
    }
}

/// The same hit, resolved by the sidecar and predicted by a browser, both ways
/// a browser has had of doing it.
///
/// Both clients are fed the authority's car 1 for the current tick, every tick:
/// a perfect, zero-latency feed. That is the control, and it is the whole point
/// of the comparison -- with the connection taken out of it, what is left is the
/// client's own arithmetic disagreeing about the contact. How much a *stale*
/// snapshot costs on top is the netcode's question, and
/// `physics::tests::a_client_predicts_the_hit_the_authority_resolves` is where
/// it gets asked.
///
/// * **parked** simulates car 0 and leaves car 1 where the feed put it. What
///   the browser used to do. The contact solver still gives the parked car its
///   real mass -- it weighs what it weighs whoever is integrating it -- but the
///   positional repair may not move it, so the client takes all of that shove
///   itself and ends up somewhere the authority never put it.
/// * **stepped** integrates both, which is what the browser does now that it
///   predicts rivals through the physics rather than extrapolating their poses.
///   Handed the same state the authority has, it does the same arithmetic on it
///   and lands on the same bits, so the error is not small: it is absent.
///
/// Reported a round trip after the hit, because that is when reconciliation
/// arrives and takes the error away. Left to run, any difference at all keeps
/// integrating and the number stops meaning anything.
fn client_vs_authority() {
    println!();
    println!("--- the same hit, as the sidecar resolves it and as a browser predicts it ---");
    println!("  closing   authority   parked client       stepped client");
    for closing in [2.0f32, 6.0, 12.0, 20.0, 30.0] {
        let (mut a, fwd) = stage(6.0, closing, 20.0);
        let mut clients = [stage(6.0, closing, 20.0).0, stage(6.0, closing, 20.0).0];
        let masks = [0b01u32, 0b11];

        let mut since_hit = -1i32;
        while since_hit < 12 {
            // The rival's snapshot for the tick the clients are about to step
            // *from*, which is the one `placeRemotes` used to fetch and the one
            // a reconcile seeds a slot with. Handing over the tick after would
            // be a favour no browser gets -- and it is a favour that stayed
            // invisible for as long as the rival was scenery, because scenery
            // does not care which tick it is standing in.
            for c in clients.iter_mut() {
                c.cars[1] = a.cars[1];
            }
            a.step(0b11);
            for (c, mask) in clients.iter_mut().zip(masks) {
                c.step(mask);
            }
            if since_hit >= 0 {
                since_hit += 1;
            } else if a.cars[0].impact > 1.0 {
                since_hit = 0;
            }
        }
        let cell = |c: &World| {
            format!(
                "{:6.2} m/s {:5.2} m",
                along(c, 0, fwd) - along(&a, 0, fwd),
                c.cars[0].pos().sub(a.cars[0].pos()).len(),
            )
        };
        println!(
            "  {closing:5.0} m/s   {:6.2} m/s   {}   {}",
            along(&a, 0, fwd),
            cell(&clients[0]),
            cell(&clients[1]),
        );
    }
}

/// How far into each other they get before anything is done about it.
fn penetration() {
    println!();
    println!("--- deepest penetration reached before the contact is resolved ---");
    println!("  closing   deepest overlap   at tick");
    for closing in [6.0f32, 15.0, 30.0, 45.0] {
        let (mut w, _) = stage(6.0, closing, 20.0);
        let mut deepest = 0.0f32;
        let mut at = 0usize;
        for t in 0..90 {
            w.step(0b11);
            let d = overlap(&w);
            if d > deepest {
                deepest = d;
                at = t;
            }
        }
        println!("  {closing:5.0} m/s   {deepest:11.3} m       {at}");
    }
}

/// Current overlap depth between the two cars, metres.
fn overlap(w: &World) -> f32 {
    use physics::car::{HALF_LEN, HALF_WID};
    use physics::collide::{box_box, Obb};
    let a = Obb::new(w.cars[0].pos(), w.cars[0].heading, HALF_LEN, HALF_WID);
    let b = Obb::new(w.cars[1].pos(), w.cars[1].heading, HALF_LEN, HALF_WID);
    match box_box(&a, &b) {
        Some(m) => {
            let mut d = 0.0f32;
            for c in m.as_slice() {
                if c.depth > d {
                    d = c.depth;
                }
            }
            d
        }
        None => 0.0,
    }
}

fn main() {
    rear_end();
    client_vs_authority();
    penetration();
}
