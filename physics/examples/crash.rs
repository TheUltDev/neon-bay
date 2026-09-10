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
//! 2. **Does the client predict the same hit the authority resolves?** The
//!    browser only simulates its own car. What it assumes about the mass of the
//!    car it just hit decides whether its prediction is nearly right or
//!    completely wrong, and the driver feels the difference immediately.
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
/// `mask` is what gets simulated: `0b11` is the sidecar, which owns both cars,
/// and `0b01` is the browser, which owns one and treats the other as scenery.
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

/// The same hit, resolved by the sidecar and predicted by a browser.
///
/// The browser runs one car and takes everyone else from the network, so the
/// client world here simulates car 0 only and overwrites car 1 from the
/// authority every tick -- a perfect, zero-latency feed, which is the right
/// control: whatever error is left is the client's own physics disagreeing
/// about the contact, not the connection.
///
/// Reported a round trip after the hit, because that is when reconciliation
/// arrives and takes the error away. Left to run, any difference at all keeps
/// integrating and the number stops meaning anything.
fn client_vs_authority() {
    println!();
    println!("--- the same hit, as the sidecar resolves it and as a browser predicts it ---");
    println!("  closing   authority   client   speed error   apart");
    for closing in [2.0f32, 6.0, 12.0, 20.0, 30.0] {
        let (mut a, fwd) = stage(6.0, closing, 20.0);
        let (mut c, _) = stage(6.0, closing, 20.0);

        let mut since_hit = -1i32;
        while since_hit < 12 {
            a.step(0b11);
            // The rival's snapshot for this tick, before the client steps: the
            // netcode interpolates remote cars up to the current tick, so a
            // client is not a tick behind on where they are.
            c.cars[1] = a.cars[1];
            c.step(0b01);
            if since_hit >= 0 {
                since_hit += 1;
            } else if a.cars[0].impact > 1.0 {
                since_hit = 0;
            }
        }
        let worst_pos = c.cars[0].pos().sub(a.cars[0].pos()).len();
        println!(
            "  {closing:5.0} m/s   {:6.2} m/s   {:5.2} m/s   {:7.2} m/s   {worst_pos:5.2} m",
            along(&a, 0, fwd),
            along(&c, 0, fwd),
            along(&c, 0, fwd) - along(&a, 0, fwd),
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
