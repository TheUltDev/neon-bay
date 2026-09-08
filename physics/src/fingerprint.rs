//! One number that says whether two builds of this crate compute the same thing.
//!
//! The demo's whole claim is that the browser and the sidecar run identical
//! instructions on identical inputs, so a prediction is not a guess. Ship a
//! client whose `physics.wasm` is a commit behind the sidecar it talks to and
//! that stops being true -- quietly. Nothing errors and nothing logs. The car
//! simply drifts off the authority's answer and gets dragged back, corner after
//! corner, and it reads as bad netcode rather than as a bad deploy.
//!
//! So neither side asserts a version, which would only tell you what someone
//! remembered to bump. Each side runs the same short scripted race and reduces
//! where it ends up to a single number. Two builds agree here if and only if
//! they agree on the arithmetic: a refactor that changes nothing observable
//! still matches, and moving one constant does not.

use crate::car::{CarInput, CAR_FLOATS, SUBSTEPS, TICK_HZ};
use crate::track::{CHECKPOINTS, SAMPLES};
use crate::world::{World, MAX_CARS};

/// Ticks of scripted racing folded into the fingerprint. Long enough for two
/// cars to corner, brake, slide and lean on each other; short enough that both
/// sides can run it at startup without anyone noticing.
const TICKS: u64 = 240;
/// Cars in the scripted race.
const CARS: usize = 2;

/// Fingerprint of what this build computes. See the module docs.
pub fn fingerprint() -> u32 {
    let mut w = World::new();
    w.spawn(0, 0);
    w.spawn(1, 3);
    for tick in 0..TICKS {
        w.inputs[0] = scripted(tick);
        w.inputs[1] = scripted(tick + 511);
        w.step((1 << CARS) - 1);
    }
    hash(&w)
}

/// Reduce a finished run to one number.
pub(crate) fn hash(w: &World) -> u32 {
    // Shape first. Changing the world size or the record layout need not move a
    // single float in a two-car race, but both sides still have to agree on it.
    let mut h = FNV_OFFSET;
    for v in [
        MAX_CARS as u32,
        CAR_FLOATS as u32,
        SAMPLES as u32,
        CHECKPOINTS as u32,
        TICK_HZ,
        SUBSTEPS,
    ] {
        h = fold(h, v);
    }
    // Then every bit of what the race ended on, including the fields nothing
    // reads back: this is looking for disagreement, not for visible difference.
    for car in &w.cars[..CARS] {
        for f in car.as_floats() {
            h = fold(h, f.to_bits());
        }
    }
    h
}

/// The scripted driver. Derived from integer arithmetic only, so the *inputs*
/// are identical on both sides by construction and the only thing under test is
/// the simulation. Same trick, and the same sequence, as `examples/trace.rs`.
fn scripted(tick: u64) -> CarInput {
    CarInput {
        throttle: if (tick / 37) % 3 == 0 { 0.0 } else { 1.0 },
        steer: ((tick % 240) as f32 - 120.0) / 120.0,
        brake: if (tick / 53) % 5 == 0 { 0.5 } else { 0.0 },
        handbrake: if (tick / 97) % 7 == 0 { 1.0 } else { 0.0 },
    }
}

const FNV_OFFSET: u32 = 0x811c_9dc5;
const FNV_PRIME: u32 = 0x0100_0193;

/// FNV-1a over four bytes. Any mixing function would do; this one is five lines
/// and pulls in nothing.
#[inline]
fn fold(mut h: u32, v: u32) -> u32 {
    for byte in v.to_le_bytes() {
        h ^= byte as u32;
        h = h.wrapping_mul(FNV_PRIME);
    }
    h
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn is_reproducible() {
        let a = fingerprint();
        assert_eq!(a, fingerprint());
        println!("physics fingerprint {a:#010x}");
    }

    /// The check is only worth having if it actually moves. One ULP in one
    /// field of one car -- far less than any real change to the simulation --
    /// has to come out the other end as a different number.
    #[test]
    fn notices_a_single_bit() {
        let mut w = World::new();
        w.spawn(0, 0);
        w.spawn(1, 3);
        let before = hash(&w);
        w.cars[0].vx = f32::from_bits(w.cars[0].vx.to_bits() + 1);
        assert_ne!(before, hash(&w), "a changed simulation hashed the same");
    }
}
