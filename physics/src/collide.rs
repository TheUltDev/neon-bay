//! Contacts: oriented boxes, a real manifold, and a sequential-impulse solver.
//!
//! A car used to be two circles. Circles are cheap and they never catch, but
//! they are also round, and a round car is wrong in ways you can feel: it
//! cannot rest flat against a barrier, two of them side by side slide past
//! instead of leaning, and every glancing blow arrives through a single point
//! on a normal that swings as the cars move.
//!
//! This is the box the car actually is. [`box_box`] separates two of them with
//! SAT, then clips one face against the other to get up to **two** contact
//! points, which is what a flat contact needs: one point can only ever push,
//! two can push *and* resist a twist. That is the difference between scraping
//! along a wall and pirouetting off it.
//!
//! [`solve`] runs the constraints as accumulated impulses -- normal impulses
//! clamped non-negative, friction clamped to the Coulomb cone against whatever
//! normal impulse that same contact has built up so far -- for a fixed number
//! of passes. Fixed count, fixed order, no sorting anywhere: the answer depends
//! only on the input bits, which is what rollback needs.

use crate::math::{abs, max, min, V2};

/// Penetration left uncorrected, meters. Contacts that are allowed to rest a
/// little inside each other stop jittering between touching and not.
const SLOP: f32 = 0.005;
/// Fraction of the remaining penetration pushed out per pass.
///
/// Small, because contacts are solved every dynamics substep rather than once
/// a tick: eight passes at a quarter each remove 90% of an overlap inside one
/// frame, which is the same authority the old single 0.8 pass had, without a
/// single pass ever being large enough to look like a shove.
const CORRECTION: f32 = 0.25;
/// Approach speed below which a contact is treated as a rest, not a bounce.
/// Without it, resting contacts jitter forever on their own restitution.
const REST_THRESHOLD: f32 = 0.6;
/// Solver passes. Enough for a car wedged between another car and a barrier.
pub const ITERATIONS: u32 = 6;

/// An oriented box: centre, unit axes, half extents.
#[derive(Clone, Copy, Debug)]
pub struct Obb {
    pub c: V2,
    /// Unit vector along the body's length.
    pub ax: V2,
    /// Unit vector along the body's width, `ax` turned 90 degrees.
    pub ay: V2,
    pub hx: f32,
    pub hy: f32,
}

impl Obb {
    pub fn new(c: V2, heading: f32, hx: f32, hy: f32) -> Obb {
        let ax = V2::from_angle(heading);
        Obb { c, ax, ay: ax.perp(), hx, hy }
    }

    /// The four corners, in a fixed order.
    pub fn corners(&self) -> [V2; 4] {
        let x = self.ax.scale(self.hx);
        let y = self.ay.scale(self.hy);
        [
            self.c.add(x).add(y),
            self.c.sub(x).add(y),
            self.c.sub(x).sub(y),
            self.c.add(x).sub(y),
        ]
    }

    /// Half the box's extent along `n`, for the SAT projection.
    #[inline]
    fn radius(&self, n: V2) -> f32 {
        self.hx * abs(self.ax.dot(n)) + self.hy * abs(self.ay.dot(n))
    }

    /// The face whose outward normal is most aligned with `d`, as its two
    /// endpoints and that normal.
    fn support_face(&self, d: V2) -> (V2, V2, V2) {
        let v = self.corners();
        // Same order as `corners`: +ay spans v0..v1, -ax spans v1..v2,
        // -ay spans v2..v3, +ax spans v3..v0.
        let faces = [
            (v[0], v[1], self.ay),
            (v[1], v[2], self.ax.scale(-1.0)),
            (v[2], v[3], self.ay.scale(-1.0)),
            (v[3], v[0], self.ax),
        ];
        let mut best = 0usize;
        let mut best_dot = faces[0].2.dot(d);
        for (i, f) in faces.iter().enumerate().skip(1) {
            let dot = f.2.dot(d);
            if dot > best_dot {
                best_dot = dot;
                best = i;
            }
        }
        faces[best]
    }
}

/// One contact point, plus the solver's accumulated state for it.
#[derive(Clone, Copy, Debug, Default)]
pub struct Constraint {
    pub point: V2,
    /// Unit normal, pointing from B towards A. Push A along `+normal`.
    pub normal: V2,
    pub depth: f32,
    /// Accumulated impulses, so the clamping is on the total rather than on
    /// each increment. This is what lets six cheap passes behave like one
    /// expensive solve.
    jn: f32,
    jt: f32,
    bias: f32,
    mass_n: f32,
    mass_t: f32,
    ra: V2,
    rb: V2,
}

/// Contacts between one pair of shapes.
///
/// Four, not two: [`box_box`] never produces more than two, but a car in a
/// narrowing section can be touching the barrier on both sides at once, and
/// solving those together is the difference between being guided through and
/// being fired out of it.
pub const MAX_CONTACTS: usize = 4;

#[derive(Clone, Copy, Debug, Default)]
pub struct Manifold {
    pub count: usize,
    pub contacts: [Constraint; MAX_CONTACTS],
}

impl Manifold {
    pub fn push(&mut self, point: V2, normal: V2, depth: f32) {
        if self.count < MAX_CONTACTS {
            self.contacts[self.count] = Constraint { point, normal, depth, ..Default::default() };
            self.count += 1;
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Constraint] {
        &mut self.contacts[..self.count]
    }

    pub fn as_slice(&self) -> &[Constraint] {
        &self.contacts[..self.count]
    }
}

impl Constraint {
    /// Normal impulse this contact ended up carrying, newton-seconds. How the
    /// damage model decides which panel a multi-point hit landed on.
    #[inline]
    pub fn impulse(&self) -> f32 {
        self.jn
    }
}

/// A rigid body, as the solver needs to see it. A static collider is one with
/// zero inverse mass and inertia, which falls out of the same arithmetic
/// without a special case.
#[derive(Clone, Copy, Debug, Default)]
pub struct Body {
    pub pos: V2,
    pub vel: V2,
    pub omega: f32,
    pub inv_m: f32,
    pub inv_i: f32,
}

impl Body {
    pub fn fixed(pos: V2) -> Body {
        Body { pos, ..Default::default() }
    }

    /// Velocity of the material point at `r` (relative to the centre of mass).
    #[inline]
    fn point_vel(&self, r: V2) -> V2 {
        V2::new(self.vel.x - self.omega * r.y, self.vel.y + self.omega * r.x)
    }

    #[inline]
    fn apply(&mut self, r: V2, imp: V2) {
        self.vel.x += imp.x * self.inv_m;
        self.vel.y += imp.y * self.inv_m;
        self.omega += r.cross(imp) * self.inv_i;
    }
}

/// Separating-axis test between two oriented boxes, with the contact manifold
/// if they overlap.
///
/// The normal points from `b` towards `a`, so `a` is the one pushed along it.
pub fn box_box(a: &Obb, b: &Obb) -> Option<Manifold> {
    let axes = [a.ax, a.ay, b.ax, b.ay];
    let delta = a.c.sub(b.c);

    let mut best = 0usize;
    let mut least = f32::MAX;
    for (i, n) in axes.iter().enumerate() {
        let overlap = a.radius(*n) + b.radius(*n) - abs(delta.dot(*n));
        if overlap <= 0.0 {
            return None;
        }
        // Strictly less-than, so a tie always keeps the earlier axis and the
        // result never depends on iteration order.
        if overlap < least {
            least = overlap;
            best = i;
        }
    }

    // Orient the chosen axis from b towards a.
    let mut n = axes[best];
    if delta.dot(n) < 0.0 {
        n = n.scale(-1.0);
    }

    // The box that owns the axis is the reference; its face points at the
    // other one.
    let (reference, incident, ref_n) = if best < 2 {
        (a, b, n.scale(-1.0))
    } else {
        (b, a, n)
    };

    let (r0, r1, rn) = reference.support_face(ref_n);
    let (i0, i1, _) = incident.support_face(rn.scale(-1.0));

    // Clip the incident face to the strip the reference face actually covers.
    let t = r1.sub(r0).normalize();
    let (c0, c1) = clip(i0, i1, t.scale(-1.0), -r0.dot(t))?;
    let (c0, c1) = clip(c0, c1, t, r1.dot(t))?;

    let mut m = Manifold::default();
    for p in [c0, c1] {
        // Distance behind the reference face is the penetration.
        let depth = -p.sub(r0).dot(rn);
        if depth >= 0.0 {
            m.push(p, n, depth);
        }
    }
    if m.count == 0 {
        None
    } else {
        Some(m)
    }
}

/// Keep the part of a segment on the `p . n <= c` side of a plane.
fn clip(p0: V2, p1: V2, n: V2, c: f32) -> Option<(V2, V2)> {
    let d0 = p0.dot(n) - c;
    let d1 = p1.dot(n) - c;
    if d0 <= 0.0 && d1 <= 0.0 {
        return Some((p0, p1));
    }
    if d0 > 0.0 && d1 > 0.0 {
        return None;
    }
    let t = d0 / (d0 - d1);
    let cut = p0.add(p1.sub(p0).scale(t));
    Some(if d0 > 0.0 { (cut, p1) } else { (p0, cut) })
}

/// Prepare a set of constraints: effective masses, and how much of the approach
/// speed is going to be given back as a bounce.
///
/// `bounce` scales the crush model's own restitution rather than replacing it,
/// so each surface says how springy it is *relative to* bending sheet metal and
/// the speed-dependence is not something every caller has to remember. See
/// [`crate::damage::restitution`]: a hard hit is nearly plastic because the
/// energy went into the shape of the car, not back into its velocity.
pub fn prepare(a: &Body, b: &Body, cs: &mut [Constraint], bounce: f32) {
    for c in cs.iter_mut() {
        c.ra = c.point.sub(a.pos);
        c.rb = c.point.sub(b.pos);
        let t = c.normal.perp();

        let rna = c.ra.cross(c.normal);
        let rnb = c.rb.cross(c.normal);
        let kn = a.inv_m + b.inv_m + rna * rna * a.inv_i + rnb * rnb * b.inv_i;
        c.mass_n = if kn > 1e-9 { 1.0 / kn } else { 0.0 };

        let rta = c.ra.cross(t);
        let rtb = c.rb.cross(t);
        let kt = a.inv_m + b.inv_m + rta * rta * a.inv_i + rtb * rtb * b.inv_i;
        c.mass_t = if kt > 1e-9 { 1.0 / kt } else { 0.0 };

        // Restitution off the *approach* speed, decided once. Reading it inside
        // the iteration loop instead would let a contact feed on itself.
        let vn = b.point_vel(c.rb).sub(a.point_vel(c.ra)).dot(c.normal);
        let e = bounce * crate::damage::restitution(vn);
        c.bias = if vn > REST_THRESHOLD { e * (vn - REST_THRESHOLD) } else { 0.0 };
        c.jn = 0.0;
        c.jt = 0.0;
    }
}

/// What a resolved contact did, as the rest of the game needs to know it.
///
/// The two energies are kept apart because they mean completely different
/// things to the bodywork, and telling them apart from the impulses alone is
/// not possible. A car resting against a barrier at speed carries a large
/// normal impulse -- that is what holds it out of the wall -- while doing no
/// crushing whatever, because nothing is moving along the normal. What folds
/// metal is normal impulse doing *work*, and that is what [`crush`] measures.
///
/// [`crush`]: Impact::crush
#[derive(Clone, Copy, Debug, Default)]
pub struct Impact {
    /// Impulse applied along the contact normal, newton-seconds.
    pub normal: f32,
    /// Impulse applied along the surface, newton-seconds.
    pub tangent: f32,
    /// Kinetic energy destroyed head-on, joules: the part that folds panels.
    pub crush: f32,
    /// Kinetic energy destroyed by sliding, joules: the part that scores them.
    pub scrape: f32,
}

/// Energy an impulse `j` along `dir` removed, given the relative velocity along
/// `dir` before and after it was applied.
///
/// For a pair of rigid bodies the work an impulse does is exactly the impulse
/// times the *mean* of the relative velocity across it, rotation included. So
/// this is not an estimate, and it goes to zero for a contact that is merely
/// holding station -- which is the whole point of measuring it.
#[inline]
fn work(j: f32, v_before: f32, v_after: f32) -> f32 {
    -j * (v_before + v_after) * 0.5
}

/// Run the impulse solver. `friction` is the Coulomb coefficient between the
/// two surfaces.
pub fn solve(a: &mut Body, b: &mut Body, cs: &mut [Constraint], friction: f32) -> Impact {
    let mut crush = 0.0;
    let mut scrape = 0.0;
    for _ in 0..ITERATIONS {
        for c in cs.iter_mut() {
            // --- normal ----------------------------------------------------
            // `normal` points b -> a, so a positive closing speed is a's point
            // moving *against* it.
            let rel = a.point_vel(c.ra).sub(b.point_vel(c.rb));
            let vn = rel.dot(c.normal);
            let mut dj = (-vn + c.bias) * c.mass_n;
            // Clamp the accumulated impulse, not the increment: a contact may
            // pull back what an earlier pass over-applied, but the total can
            // never go negative and start sucking the bodies together.
            let old = c.jn;
            c.jn = max(old + dj, 0.0);
            dj = c.jn - old;
            let imp = c.normal.scale(dj);
            a.apply(c.ra, imp);
            b.apply(c.rb, imp.scale(-1.0));

            // --- friction ---------------------------------------------------
            let t = c.normal.perp();
            let rel = a.point_vel(c.ra).sub(b.point_vel(c.rb));
            crush += work(dj, vn, rel.dot(c.normal));
            let vt = rel.dot(t);
            let mut dj = -vt * c.mass_t;
            let cap = friction * c.jn;
            let old = c.jt;
            c.jt = crate::math::clamp(old + dj, -cap, cap);
            dj = c.jt - old;
            let imp = t.scale(dj);
            a.apply(c.ra, imp);
            b.apply(c.rb, imp.scale(-1.0));
            scrape += work(dj, vt, a.point_vel(c.ra).sub(b.point_vel(c.rb)).dot(t));
        }
    }

    let mut normal = 0.0;
    let mut tangent = 0.0;
    for c in cs.iter() {
        normal += c.jn;
        tangent += abs(c.jt);
    }
    Impact { normal, tangent, crush: max(crush, 0.0), scrape: max(scrape, 0.0) }
}

impl Impact {
    /// How hard the hit felt, newton-seconds. Sliding along a wall is a real
    /// impulse but it is not a crash, so the tangential half counts for less.
    #[inline]
    pub fn severity(&self) -> f32 {
        self.normal + self.tangent * 0.5
    }
}

/// Push overlapping bodies apart, split by inverse mass.
///
/// Deliberately translation only. Position correction is a numerical repair,
/// not a force, and letting it rotate things puts spin into the world that no
/// impulse paid for.
///
/// Contacts are grouped by normal before being applied, so the two points of
/// one flat face push once between them, while a car genuinely wedged against
/// two different surfaces still gets pushed out of both.
pub fn separate(a: &mut Body, b: &mut Body, cs: &[Constraint]) {
    let total = a.inv_m + b.inv_m;
    if total <= 0.0 {
        return;
    }
    for (i, c) in cs.iter().enumerate() {
        // Only the deepest contact of each distinct normal acts. Earlier
        // contacts win ties, so the choice never depends on iteration order.
        let mut redundant = false;
        for (k, other) in cs.iter().enumerate() {
            if k == i || other.normal.dot(c.normal) < 0.98 {
                continue;
            }
            if other.depth > c.depth || (other.depth == c.depth && k < i) {
                redundant = true;
                break;
            }
        }
        if redundant {
            continue;
        }
        let push = max(c.depth - SLOP, 0.0) * CORRECTION;
        if push <= 0.0 {
            continue;
        }
        let d = c.normal.scale(push / total);
        a.pos = a.pos.add(d.scale(a.inv_m));
        b.pos = b.pos.sub(d.scale(b.inv_m));
    }
}

/// Clamp a body's spin. A car caught between a barrier and another car can be
/// handed more angular velocity in one tick than anything on track should have.
#[inline]
pub fn clamp_spin(omega: f32, limit: f32) -> f32 {
    min(max(omega, -limit), limit)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::PI;

    fn body(pos: V2) -> Body {
        Body { pos, inv_m: 1.0 / 1150.0, inv_i: 1.0 / 1600.0, ..Default::default() }
    }

    #[test]
    fn boxes_apart_do_not_touch() {
        let a = Obb::new(V2::new(0.0, 0.0), 0.0, 2.1, 0.95);
        let b = Obb::new(V2::new(6.0, 0.0), 0.0, 2.1, 0.95);
        assert!(box_box(&a, &b).is_none());
    }

    /// Two cars nose to tail should meet on a flat face, which means two
    /// contact points and a normal along the length of the car.
    #[test]
    fn a_flat_contact_gives_two_points() {
        let a = Obb::new(V2::new(0.0, 0.0), 0.0, 2.1, 0.95);
        let b = Obb::new(V2::new(4.0, 0.0), 0.0, 2.1, 0.95);
        let m = box_box(&a, &b).expect("overlapping boxes reported no contact");
        println!(
            "nose to tail: {} points, normal ({:.2}, {:.2}), depth {:.3}",
            m.count, m.contacts[0].normal.x, m.contacts[0].normal.y, m.contacts[0].depth
        );
        assert_eq!(m.count, 2, "a flat face gave {} contact point(s)", m.count);
        // Normal points b -> a, so a is pushed back along -x.
        assert!(m.contacts[0].normal.x < -0.99);
        assert!(abs(m.contacts[0].depth - 0.2) < 1e-3);
    }

    /// A corner into a flat side is a genuine one-point contact.
    #[test]
    fn a_corner_contact_gives_one_point() {
        let a = Obb::new(V2::new(0.0, 0.0), 0.0, 2.1, 0.95);
        let b = Obb::new(V2::new(2.9, 1.6), PI / 4.0, 2.1, 0.95);
        let m = box_box(&a, &b).expect("overlapping boxes reported no contact");
        println!("corner hit: {} point(s)", m.count);
        assert!(m.count >= 1 && m.count <= 2);
    }

    #[test]
    fn separation_is_symmetric_in_the_axis_order() {
        let a = Obb::new(V2::new(0.0, 0.0), 0.3, 2.1, 0.95);
        let b = Obb::new(V2::new(3.4, 0.9), -0.2, 2.1, 0.95);
        let ab = box_box(&a, &b).expect("no contact a->b");
        let ba = box_box(&b, &a).expect("no contact b->a");
        // Same penetration either way round, and opposite normals.
        let da = ab.contacts[0].depth;
        let db = ba.contacts[0].depth;
        println!("depth a->b {da:.4}, b->a {db:.4}");
        assert!(abs(da - db) < 1e-3, "penetration depends on argument order");
        assert!(ab.contacts[0].normal.dot(ba.contacts[0].normal) < -0.99);
    }

    /// The solver must remove the closing speed and never add any.
    #[test]
    fn impulses_stop_the_approach_without_adding_energy() {
        let a_box = Obb::new(V2::new(0.0, 0.0), 0.0, 2.1, 0.95);
        let b_box = Obb::new(V2::new(4.0, 0.0), 0.0, 2.1, 0.95);
        let mut m = box_box(&a_box, &b_box).unwrap();
        let mut a = body(a_box.c);
        let mut b = body(b_box.c);
        a.vel = V2::new(12.0, 0.0);

        let before = 0.5 * 1150.0 * a.vel.len_sq();
        prepare(&a, &b, m.as_slice_mut(), 0.0);
        solve(&mut a, &mut b, m.as_slice_mut(), 0.4);
        let after = 0.5 * 1150.0 * (a.vel.len_sq() + b.vel.len_sq());

        println!(
            "12 m/s into a stationary car: a {:.2} m/s, b {:.2} m/s",
            a.vel.x, b.vel.x
        );
        assert!(a.vel.x < b.vel.x + 1e-3, "cars are still closing");
        assert!(after <= before + 1.0, "solver added energy: {before:.0} -> {after:.0} J");
        // Momentum is conserved, which is the real check.
        let p = 1150.0 * (a.vel.x + b.vel.x);
        assert!(abs(p - 1150.0 * 12.0) < 1.0, "momentum drifted to {p:.0}");
    }

    /// Restitution has to give some of it back, a resting contact none, and a
    /// hard hit proportionally less than a soft one -- which is the crush
    /// model's whole claim about how a car behaves.
    #[test]
    fn restitution_bounces_but_resting_contacts_do_not() {
        let mk = || {
            let a_box = Obb::new(V2::new(0.0, 0.0), 0.0, 2.1, 0.95);
            let b_box = Obb::new(V2::new(4.0, 0.0), 0.0, 2.1, 0.95);
            box_box(&a_box, &b_box).unwrap()
        };

        let mut m = mk();
        let mut a = body(V2::ZERO);
        let mut b = body(V2::new(4.0, 0.0));
        a.vel = V2::new(10.0, 0.0);
        prepare(&a, &b, m.as_slice_mut(), 0.5);
        solve(&mut a, &mut b, m.as_slice_mut(), 0.4);
        let fast = b.vel.x - a.vel.x;

        let mut m = mk();
        let mut a = body(V2::ZERO);
        let mut b = body(V2::new(4.0, 0.0));
        a.vel = V2::new(0.2, 0.0);
        prepare(&a, &b, m.as_slice_mut(), 0.5);
        solve(&mut a, &mut b, m.as_slice_mut(), 0.4);
        let slow = b.vel.x - a.vel.x;

        let mut m = mk();
        let mut a = body(V2::ZERO);
        let mut b = body(V2::new(4.0, 0.0));
        a.vel = V2::new(3.0, 0.0);
        prepare(&a, &b, m.as_slice_mut(), 0.5);
        solve(&mut a, &mut b, m.as_slice_mut(), 0.4);
        let gentle = b.vel.x - a.vel.x;

        println!(
            "separation: 10 m/s hit -> {fast:.2} m/s ({:.0}%), 3 m/s nudge -> {gentle:.2} m/s ({:.0}%), 0.2 m/s touch -> {slow:.3}",
            fast / 10.0 * 100.0,
            gentle / 3.0 * 100.0
        );
        assert!(fast > 0.3, "no bounce at all from a 10 m/s hit");
        assert!(slow < 0.05, "a resting contact bounced at {slow:.3} m/s");
        // The point of the crush model: the harder it is hit, the less of it
        // comes back, because the rest went into the shape of the car.
        assert!(
            gentle / 3.0 > fast / 10.0 * 1.5,
            "a hard hit is as elastic as a soft one, which is not how metal works"
        );
    }

    /// A contact merely holding a body in place must not report crushing it,
    /// however large the impulse it takes to do that.
    #[test]
    fn a_resting_contact_crushes_nothing() {
        let a_box = Obb::new(V2::new(0.0, 0.0), 0.0, 2.1, 0.95);
        let b_box = Obb::new(V2::new(4.19, 0.0), 0.0, 2.1, 0.95);
        let mut m = box_box(&a_box, &b_box).unwrap();
        let mut a = body(a_box.c);
        let mut b = Body::fixed(b_box.c);
        // Sliding along the face at speed, barely moving into it.
        a.vel = V2::new(0.02, 30.0);
        prepare(&a, &b, m.as_slice_mut(), 1.0);
        let hit = solve(&mut a, &mut b, m.as_slice_mut(), 0.55);
        println!(
            "scraping past at 30 m/s: {:.0} N.s normal, {:.0} N.s tangent, {:.1} J crush, {:.0} J scrape",
            hit.normal,
            hit.tangent,
            hit.crush,
            hit.scrape
        );
        assert!(hit.scrape > 300.0, "a scrape at 30 m/s dissipated nothing");
        assert!(
            hit.crush < hit.scrape * 0.02,
            "a scrape was charged {:.0} J of crushing",
            hit.crush
        );
    }

    /// Two points on a face have to resist a twist, which one point cannot.
    #[test]
    fn a_two_point_contact_resists_rotation() {
        let a_box = Obb::new(V2::new(0.0, 0.0), 0.0, 2.1, 0.95);
        let b_box = Obb::new(V2::new(4.0, 0.0), 0.0, 2.1, 0.95);
        let mut m = box_box(&a_box, &b_box).unwrap();
        assert_eq!(m.count, 2);
        let mut a = body(a_box.c);
        let mut b = body(b_box.c);
        a.omega = 3.0;
        prepare(&a, &b, m.as_slice_mut(), 0.0);
        solve(&mut a, &mut b, m.as_slice_mut(), 0.8);
        println!("spin into a flat contact: {:.2} -> {:.2} rad/s", 3.0, a.omega);
        assert!(a.omega < 2.4, "flat contact did nothing about a spin");
    }

    #[test]
    fn separation_pushes_only_the_movable_body() {
        let a_box = Obb::new(V2::new(0.0, 0.0), 0.0, 2.1, 0.95);
        let b_box = Obb::new(V2::new(4.0, 0.0), 0.0, 2.1, 0.95);
        let m = box_box(&a_box, &b_box).unwrap();
        let mut a = body(a_box.c);
        let mut b = Body::fixed(b_box.c);
        separate(&mut a, &mut b, m.as_slice());
        println!("a pushed to {:.4}, static b at {:.4}", a.pos.x, b.pos.x);
        assert!(a.pos.x < 0.0, "movable body was not pushed clear");
        assert_eq!(b.pos.x, 4.0, "static body moved");
    }
}
