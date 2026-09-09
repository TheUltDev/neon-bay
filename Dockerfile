# syntax=docker/dockerfile:1

# The database and its authority, in one container.
#
# SpacetimeDB and the sidecar are two processes, but putting them on two Railway
# services would put a network hop on the hot path: the sidecar reads every
# input as it lands and writes twenty snapshots a second, at a fixed 60 Hz it
# cannot afford to miss. Same container means that hop is loopback, which is as
# close to free as it gets and takes a variable this demo is trying to *measure*
# out of the measurement.
#
# The version below is the fourth place SpacetimeDB is pinned. It moves together
# with module/Cargo.toml, sidecar/Cargo.toml and web/package.json.
ARG SPACETIME_VERSION=v2.8.2

# ------------------------------------------------------------------ build --
# The official image is `rust:bookworm` plus the wasm target, binaryen and the
# CLI, so it already has every tool this repo needs. Building in it rather than
# in a stock Rust image also means the sidecar links against exactly the glibc
# it will run on.
FROM clockworklabs/spacetime:${SPACETIME_VERSION} AS builder
USER root
WORKDIR /src

COPY rust-toolchain.toml Cargo.toml Cargo.lock ./
COPY physics/ physics/
COPY sidecar/ sidecar/
COPY module/ module/

# Order matters: the module is compiled first because `spacetime generate` reads
# the schema out of the finished wasm, and the sidecar cannot compile until
# those bindings exist. They are gitignored on purpose -- generating them here,
# from the module being shipped in this same image, is what stops the two from
# drifting apart.
#
# Deliberately no `--mount=type=cache` on the cargo builds. Railway requires
# every cache mount id to be literally prefixed `s/<service id>-`, and the flag
# does not expand variables, so honouring it means hardcoding one Railway
# service into this file and handing anyone who forks the repo a build failure.
# A cold compile of a workspace this size is a few minutes and deploys are rare.
RUN mkdir -p /out \
 && spacetime build --module-path module \
 && wasm=module/target/wasm32-unknown-unknown/release/physics_sidecar_module.opt.wasm \
 && [ -f "$wasm" ] || wasm=module/target/wasm32-unknown-unknown/release/physics_sidecar_module.wasm \
 && cp "$wasm" /out/module.wasm

RUN spacetime generate --lang rust --yes \
      --bin-path /out/module.wasm \
      --out-dir sidecar/src/module_bindings

RUN cargo build --release -p sidecar \
 && cp target/release/sidecar /out/sidecar

# ---------------------------------------------------------------- runtime --
# The official image again, rather than a slim base with the two server binaries
# copied into it. That would cut a few gigabytes, and it very likely works --
# `spacetime start` finds `spacetimedb-standalone` as a sibling of itself, so
# the layout is the only thing that has to survive -- but the sidecar links
# OpenSSL and the server was built against this exact userland, and neither is
# worth guessing at from a machine that cannot run the image. Railway caches
# base layers between deploys, so the cost lands once.
FROM clockworklabs/spacetime:${SPACETIME_VERSION}

USER root
# Where the Railway volume gets mounted, and everything that has to outlive a
# deploy lives under it: the database in `data/`, the keypair identities are
# signed with in `keys/`, and the CLI's own identity -- the one that owns the
# database and is therefore the only one allowed to publish to it again -- in
# `cli.toml`. Three separate things rather than one directory, because the
# database is the only one it is ever right to throw away.
RUN mkdir -p /stdb/data && chown -R spacetime:spacetime /stdb

COPY --from=builder --chmod=755 /out/sidecar /usr/local/bin/sidecar
COPY --from=builder /out/module.wasm /app/module.wasm
COPY --chmod=755 scripts/railway-start.sh /usr/local/bin/railway-start

USER spacetime
WORKDIR /app

ENV STDB_STATE_DIR=/stdb \
    STDB_DB=physics-sidecar \
    SIDECAR_BOTS=6

EXPOSE 3000

# The base image points ENTRYPOINT at the CLI. This container is two processes,
# so it needs a supervisor instead.
ENTRYPOINT ["railway-start"]
