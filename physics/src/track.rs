//! The circuit: "Neon Bay".
//!
//! The centerline is a closed polar curve `r(theta)` built from three harmonics.
//! That shape is star-shaped by construction, so it can never self-intersect and
//! the walls can never pinch -- handy when geometry is generated, not authored.
//!
//! The curve is then *resampled at uniform arc length*, which is what makes the
//! rest of the simulation cheap: `index == s / ds`, so mapping between a car's
//! distance-along-the-lap and a wall segment is a multiply, not a search.

use crate::math::{abs, atan2, clamp, cos, sin, sqrt, wrap_pi, TAU, V2};

/// Number of centerline samples. ~1.9 m apart on a 1425 m lap.
pub const SAMPLES: usize = 768;
/// Lap is divided into this many checkpoints; all must be crossed in order.
pub const CHECKPOINTS: usize = 12;

const R0: f32 = 200.0;
const A3: f32 = 0.18;
const A5: f32 = 0.085;
const A7: f32 = 0.05;
const PH7: f32 = 0.70;

/// Resolution of the intermediate curve used for arc-length integration.
const FINE: usize = 8192;

/// Sentinel meaning "no idea where this car is, do a full search".
pub const NO_HINT: u16 = u16::MAX;

#[derive(Clone, Copy, Debug, Default)]
pub struct Hit {
    /// Index of the centerline segment the car is on.
    pub idx: u16,
    /// Distance traveled along the lap, in meters, `[0, length)`.
    pub s: f32,
    /// Signed lateral offset from the centerline; positive is track-left.
    pub lat: f32,
    /// Closest point on the centerline.
    pub point: V2,
    /// Left normal at that point.
    pub normal: V2,
    /// Half width of the track there.
    pub half_width: f32,
}

pub struct Track {
    pub p: [V2; SAMPLES],
    pub tangent: [V2; SAMPLES],
    pub normal: [V2; SAMPLES],
    /// Arc length at each sample. Uniform, so `s[i] == i * ds`.
    pub s: [f32; SAMPLES],
    /// Signed curvature (1/radius) -- bots use it to pick a corner speed.
    pub curvature: [f32; SAMPLES],
    pub half_width: [f32; SAMPLES],
    pub length: f32,
    pub ds: f32,
}

#[inline]
fn radius(t: f32) -> f32 {
    R0 * (1.0 + A3 * sin(3.0 * t) + A5 * cos(5.0 * t) + A7 * sin(7.0 * t + PH7))
}

#[inline]
fn curve_point(t: f32) -> V2 {
    let r = radius(t);
    V2::new(r * cos(t), r * sin(t))
}

impl Default for Track {
    fn default() -> Self {
        Self::new()
    }
}

impl Track {
    pub fn new() -> Self {
        // 1. Walk the analytic curve at high resolution, accumulating arc length.
        let mut fine = vec![V2::ZERO; FINE + 1];
        let mut acc = vec![0.0f32; FINE + 1];
        for (i, f) in fine.iter_mut().enumerate() {
            *f = curve_point(i as f32 / FINE as f32 * TAU);
        }
        for i in 1..=FINE {
            acc[i] = acc[i - 1] + fine[i].sub(fine[i - 1]).len();
        }
        let length = acc[FINE];
        let ds = length / SAMPLES as f32;

        // 2. Resample uniformly in arc length.
        let mut p = [V2::ZERO; SAMPLES];
        let mut cursor = 0usize;
        for (i, out) in p.iter_mut().enumerate() {
            let target = i as f32 * ds;
            while cursor + 1 < FINE && acc[cursor + 1] < target {
                cursor += 1;
            }
            let seg = acc[cursor + 1] - acc[cursor];
            let t = if seg > 1e-6 {
                (target - acc[cursor]) / seg
            } else {
                0.0
            };
            *out = V2::new(
                fine[cursor].x + (fine[cursor + 1].x - fine[cursor].x) * t,
                fine[cursor].y + (fine[cursor + 1].y - fine[cursor].y) * t,
            );
        }

        let mut t = Track {
            p,
            tangent: [V2::ZERO; SAMPLES],
            normal: [V2::ZERO; SAMPLES],
            s: [0.0; SAMPLES],
            curvature: [0.0; SAMPLES],
            half_width: [0.0; SAMPLES],
            length,
            ds,
        };
        t.rebuild_frames();

        // 3. Put the start/finish line in the middle of the longest straight,
        //    so the grid always forms up somewhere sensible.
        let start = t.straightest_index();
        if start != 0 {
            t.rotate(start);
        }

        // 4. Wider on the straights, tighter through the corners.
        for i in 0..SAMPLES {
            let r = 1.0 / (abs(t.curvature[i]) + 1e-4);
            let openness = clamp((r - 45.0) / 180.0, 0.0, 1.0);
            t.half_width[i] = 7.6 + 2.6 * openness;
        }
        t.smooth_width();
        t
    }

    fn rebuild_frames(&mut self) {
        for i in 0..SAMPLES {
            self.s[i] = i as f32 * self.ds;
            let prev = self.p[(i + SAMPLES - 1) % SAMPLES];
            let next = self.p[(i + 1) % SAMPLES];
            let tan = next.sub(prev).normalize();
            self.tangent[i] = tan;
            self.normal[i] = tan.perp();
        }
        for i in 0..SAMPLES {
            let a0 = self.tangent[(i + SAMPLES - 1) % SAMPLES];
            let a1 = self.tangent[(i + 1) % SAMPLES];
            let d = wrap_pi(atan2(a1.y, a1.x) - atan2(a0.y, a0.x));
            self.curvature[i] = d / (2.0 * self.ds);
        }
    }

    /// Index whose surrounding ~60 m is the flattest.
    fn straightest_index(&self) -> usize {
        let win = (30.0 / self.ds) as usize;
        let mut best = 0usize;
        let mut best_sum = f32::MAX;
        for i in 0..SAMPLES {
            let mut sum = 0.0;
            for k in 0..=(2 * win) {
                sum += abs(self.curvature[(i + SAMPLES + k - win) % SAMPLES]);
            }
            if sum < best_sum {
                best_sum = sum;
                best = i;
            }
        }
        best
    }

    fn rotate(&mut self, by: usize) {
        let mut p = [V2::ZERO; SAMPLES];
        for (i, out) in p.iter_mut().enumerate() {
            *out = self.p[(i + by) % SAMPLES];
        }
        self.p = p;
        self.rebuild_frames();
    }

    fn smooth_width(&mut self) {
        for _ in 0..24 {
            let mut out = [0.0f32; SAMPLES];
            for i in 0..SAMPLES {
                let a = self.half_width[(i + SAMPLES - 1) % SAMPLES];
                let b = self.half_width[i];
                let c = self.half_width[(i + 1) % SAMPLES];
                out[i] = (a + 2.0 * b + c) * 0.25;
            }
            self.half_width = out;
        }
    }

    #[inline]
    pub fn wrap_s(&self, s: f32) -> f32 {
        let mut v = s;
        while v < 0.0 {
            v += self.length;
        }
        while v >= self.length {
            v -= self.length;
        }
        v
    }

    /// Centerline pose at an arbitrary arc length (used by the bot driver).
    pub fn sample(&self, s: f32) -> (V2, V2, f32) {
        let s = self.wrap_s(s);
        let f = s / self.ds;
        let i = (f as usize) % SAMPLES;
        let j = (i + 1) % SAMPLES;
        let t = f - (f as usize) as f32;
        let pos = V2::new(
            self.p[i].x + (self.p[j].x - self.p[i].x) * t,
            self.p[i].y + (self.p[j].y - self.p[i].y) * t,
        );
        let tan = V2::new(
            self.tangent[i].x + (self.tangent[j].x - self.tangent[i].x) * t,
            self.tangent[i].y + (self.tangent[j].y - self.tangent[i].y) * t,
        )
        .normalize();
        let curv = self.curvature[i] + (self.curvature[j] - self.curvature[i]) * t;
        (pos, tan, curv)
    }

    /// Closest point on the centerline.
    ///
    /// `hint` is the previous result's index: cars move at most ~1.2 m per tick,
    /// so a 49-sample (~90 m) window is always enough, which turns this into an
    /// O(1) lookup. Pass [`NO_HINT`] after a teleport.
    pub fn nearest(&self, q: V2, hint: u16) -> Hit {
        let (from, span) = if hint == NO_HINT || hint as usize >= SAMPLES {
            (0usize, SAMPLES)
        } else {
            ((hint as usize + SAMPLES - 24) % SAMPLES, 49usize)
        };

        let mut best_i = 0usize;
        let mut best_t = 0.0f32;
        let mut best_d2 = f32::MAX;
        let mut best_pt = V2::ZERO;

        for k in 0..span {
            let i = (from + k) % SAMPLES;
            let a = self.p[i];
            let b = self.p[(i + 1) % SAMPLES];
            let ab = b.sub(a);
            let len2 = ab.len_sq();
            let t = if len2 > 1e-9 {
                clamp(q.sub(a).dot(ab) / len2, 0.0, 1.0)
            } else {
                0.0
            };
            let pt = a.add(ab.scale(t));
            let d2 = q.sub(pt).len_sq();
            if d2 < best_d2 {
                best_d2 = d2;
                best_i = i;
                best_t = t;
                best_pt = pt;
            }
        }

        let j = (best_i + 1) % SAMPLES;
        let n = V2::new(
            self.normal[best_i].x + (self.normal[j].x - self.normal[best_i].x) * best_t,
            self.normal[best_i].y + (self.normal[j].y - self.normal[best_i].y) * best_t,
        )
        .normalize();
        let hw =
            self.half_width[best_i] + (self.half_width[j] - self.half_width[best_i]) * best_t;

        Hit {
            idx: best_i as u16,
            s: self.wrap_s(self.s[best_i] + best_t * self.ds),
            lat: q.sub(best_pt).dot(n),
            point: best_pt,
            normal: n,
            half_width: hw,
        }
    }

    /// Arc length of checkpoint `k`. Checkpoint 0 is the start/finish line.
    #[inline]
    pub fn checkpoint_s(&self, k: usize) -> f32 {
        self.length * (k as f32) / (CHECKPOINTS as f32)
    }

    /// Starting grid: pairs of cars staggered back from the line.
    pub fn grid_slot(&self, slot: usize) -> (V2, f32) {
        let row = (slot / 2) as f32;
        let side = if slot % 2 == 0 { 1.0 } else { -1.0 };
        let s = self.wrap_s(self.length - 8.0 - row * 8.0);
        let (pos, tan, _) = self.sample(s);
        let n = tan.perp();
        (pos.add(n.scale(side * 3.4)), atan2(tan.y, tan.x))
    }

    /// Axis-aligned bounds of the drivable surface, for minimaps and cameras.
    pub fn bounds(&self) -> (V2, V2) {
        let mut lo = V2::new(f32::MAX, f32::MAX);
        let mut hi = V2::new(f32::MIN, f32::MIN);
        for i in 0..SAMPLES {
            let w = self.half_width[i];
            for sgn in [-1.0f32, 1.0] {
                let e = self.p[i].add(self.normal[i].scale(sgn * w));
                lo = V2::new(lo.x.min(e.x), lo.y.min(e.y));
                hi = V2::new(hi.x.max(e.x), hi.y.max(e.y));
            }
        }
        (lo, hi)
    }

    /// Braking-limited speed profile: the fastest you can be going *now* and
    /// still make every corner within `ahead` meters.
    ///
    /// Taking the minimum of the raw corner speeds instead would have the car
    /// crawling down the straights, because a hairpin 150 m away would already
    /// be capping it. Folding in `v^2 = v_corner^2 + 2*a*d` lets it stay flat
    /// out until the braking point actually arrives.
    pub fn speed_limit(&self, s: f32, ahead: f32, grip: f32) -> f32 {
        const TOP: f32 = 120.0;
        let a_brake = grip * 9.81 * 0.85;
        let steps = 22;
        let mut best = TOP;
        for i in 0..steps {
            let d = ahead * (i as f32) / (steps as f32);
            let (_, _, c) = self.sample(s + d);
            let ac = abs(c);
            let v_corner = if ac < 1e-4 { TOP } else { sqrt(grip * 9.81 / ac) };
            let allowed = sqrt(v_corner * v_corner + 2.0 * a_brake * d);
            if allowed < best {
                best = allowed;
            }
        }
        best.min(TOP)
    }
}
