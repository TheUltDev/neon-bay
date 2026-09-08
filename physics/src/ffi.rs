//! The browser bridge.
//!
//! Deliberately a bare C ABI over `f32` buffers -- no wasm-bindgen, no
//! wasm-pack, no npm plugin. `cargo build --target wasm32-unknown-unknown` emits
//! a ~40 KB `.wasm` that the client loads with `WebAssembly.instantiate`.
//!
//! Car state and inputs are handed to JavaScript as raw pointers into wasm
//! linear memory. Since [`CarState`] is `#[repr(C)]` and entirely `f32`, the
//! client maps them with `Float32Array` views and reads/writes the simulation
//! in place -- no serialization on the hot path.
//!
//! # Memory growth
//! All allocation happens inside [`phys_init`]. Nothing after it allocates, so
//! the `ArrayBuffer` backing those views is never detached by a memory grow.

use crate::car::{CarState, CAR_FLOATS, INPUT_FLOATS};
use crate::track::{Track, CHECKPOINTS, SAMPLES};
use crate::world::{World, MAX_CARS};

/// Floats per centerline sample in the exported track buffer:
/// `x, y, tangent.x, tangent.y, half_width, curvature`.
pub const TRACK_STRIDE: usize = 6;

struct Bridge {
    world: Box<World>,
    track_buf: Vec<f32>,
    consts: Vec<f32>,
    /// One car's worth of staging space, so the client can hand a snapshot in
    /// or read one out without needing an allocator of its own.
    scratch: Vec<f32>,
}

static mut BRIDGE: Option<Bridge> = None;

#[allow(static_mut_refs)]
fn bridge() -> &'static mut Bridge {
    unsafe {
        if BRIDGE.is_none() {
            let world = Box::new(World::new());
            let track_buf = flatten_track(&world.track);
            let consts = constants(&world.track);
            BRIDGE = Some(Bridge {
                world,
                track_buf,
                consts,
                scratch: vec![0.0; CAR_FLOATS],
            });
        }
        BRIDGE.as_mut().unwrap()
    }
}

fn flatten_track(t: &Track) -> Vec<f32> {
    let mut out = Vec::with_capacity(SAMPLES * TRACK_STRIDE);
    for i in 0..SAMPLES {
        out.push(t.p[i].x);
        out.push(t.p[i].y);
        out.push(t.tangent[i].x);
        out.push(t.tangent[i].y);
        out.push(t.half_width[i]);
        out.push(t.curvature[i]);
    }
    out
}

fn constants(t: &Track) -> Vec<f32> {
    vec![
        crate::car::HALF_LEN,
        crate::car::HALF_WID,
        crate::car::CIRCLE_OFF,
        crate::car::CIRCLE_R,
        crate::car::DT,
        crate::car::TICK_HZ as f32,
        t.length,
        t.ds,
        SAMPLES as f32,
        CHECKPOINTS as f32,
        MAX_CARS as f32,
        CAR_FLOATS as f32,
    ]
}

/// Build the world. Safe to call repeatedly; only the first call allocates.
#[no_mangle]
pub extern "C" fn phys_init() -> u32 {
    bridge().world.cars.len() as u32
}

/// Pointer to `MAX_CARS * CAR_FLOATS` floats of car state.
#[no_mangle]
pub extern "C" fn phys_cars_ptr() -> *mut f32 {
    bridge().world.cars.as_mut_ptr() as *mut f32
}

/// Pointer to `MAX_CARS * INPUT_FLOATS` floats of per-car input.
#[no_mangle]
pub extern "C" fn phys_inputs_ptr() -> *mut f32 {
    bridge().world.inputs.as_mut_ptr() as *mut f32
}

/// Pointer to `SAMPLES * TRACK_STRIDE` floats describing the centerline.
#[no_mangle]
pub extern "C" fn phys_track_ptr() -> *const f32 {
    bridge().track_buf.as_ptr()
}

/// Pointer to the constants block; see [`constants`] for the layout.
#[no_mangle]
pub extern "C" fn phys_consts_ptr() -> *const f32 {
    bridge().consts.as_ptr()
}

#[no_mangle]
pub extern "C" fn phys_consts_len() -> u32 {
    bridge().consts.len() as u32
}

#[no_mangle]
pub extern "C" fn phys_set_active(mask: u32) {
    bridge().world.active = mask;
}

#[no_mangle]
pub extern "C" fn phys_active() -> u32 {
    bridge().world.active
}

/// Advance one tick, integrating only the cars in `sim_mask`.
#[no_mangle]
pub extern "C" fn phys_step(sim_mask: u32) {
    bridge().world.step(sim_mask);
}

#[no_mangle]
pub extern "C" fn phys_spawn(index: u32, grid_slot: u32) {
    bridge().world.spawn(index as usize, grid_slot as usize);
}

#[no_mangle]
pub extern "C" fn phys_despawn(index: u32) {
    bridge().world.despawn(index as usize);
}

#[no_mangle]
pub extern "C" fn phys_respawn(index: u32) {
    bridge().world.respawn_in_place(index as usize);
}

/// The tick counter is `f64` across the boundary: JavaScript has no `u64`, and
/// `f64` is exact well past any plausible session length.
#[no_mangle]
pub extern "C" fn phys_set_tick(tick: f64) {
    bridge().world.tick = tick as u64;
}

#[no_mangle]
pub extern "C" fn phys_tick() -> f64 {
    bridge().world.tick as f64
}

/// Recompute a car's cached track position after JavaScript writes a pose in
/// (for example after applying an authoritative snapshot).
#[no_mangle]
pub extern "C" fn phys_reproject(index: u32) {
    let b = bridge();
    let i = index as usize;
    if i >= MAX_CARS {
        return;
    }
    let pos = b.world.cars[i].pos();
    let hit = b.world.track.nearest(pos, crate::track::NO_HINT);
    b.world.cars[i].seg = hit.idx as f32;
    b.world.cars[i].s = hit.s;
    b.world.cars[i].lat = hit.lat;
}

/// Snapshot of one car into the scratch buffer, for rollback bookkeeping.
#[no_mangle]
pub extern "C" fn phys_get(index: u32, out: *mut f32) {
    let b = bridge();
    let i = index as usize;
    if i >= MAX_CARS || out.is_null() {
        return;
    }
    unsafe {
        core::ptr::copy_nonoverlapping(
            &b.world.cars[i] as *const CarState as *const f32,
            out,
            CAR_FLOATS,
        );
    }
}

#[no_mangle]
pub extern "C" fn phys_set(index: u32, src: *const f32) {
    let b = bridge();
    let i = index as usize;
    if i >= MAX_CARS || src.is_null() {
        return;
    }
    unsafe {
        core::ptr::copy_nonoverlapping(
            src,
            &mut b.world.cars[i] as *mut CarState as *mut f32,
            CAR_FLOATS,
        );
    }
}

/// Staging buffer of [`CAR_FLOATS`] floats, for use with [`phys_get`] and
/// [`phys_set`].
#[no_mangle]
pub extern "C" fn phys_scratch_ptr() -> *mut f32 {
    bridge().scratch.as_mut_ptr()
}

#[no_mangle]
pub extern "C" fn phys_car_floats() -> u32 {
    CAR_FLOATS as u32
}

#[no_mangle]
pub extern "C" fn phys_input_floats() -> u32 {
    INPUT_FLOATS as u32
}

#[no_mangle]
pub extern "C" fn phys_max_cars() -> u32 {
    MAX_CARS as u32
}

#[no_mangle]
pub extern "C" fn phys_track_samples() -> u32 {
    SAMPLES as u32
}

#[no_mangle]
pub extern "C" fn phys_track_stride() -> u32 {
    TRACK_STRIDE as u32
}

/// Grid pose for a slot, written as `[x, y, heading]`.
#[no_mangle]
pub extern "C" fn phys_grid_slot(slot: u32, out: *mut f32) {
    let b = bridge();
    if out.is_null() {
        return;
    }
    let (p, h) = b.world.track.grid_slot(slot as usize);
    unsafe {
        *out = p.x;
        *out.add(1) = p.y;
        *out.add(2) = h;
    }
}
