// Build-time configuration, substituted by Vite from `VITE_`-prefixed
// environment variables. tsconfig deliberately keeps `types: []`, so the two
// the client actually reads are declared here rather than pulling in the whole
// of `vite/client` for them.

interface ImportMetaEnv {
  /** Where the built client looks for SpacetimeDB, e.g. `https://x.up.railway.app`. */
  readonly VITE_STDB_URI?: string;
  /** Database name, when it is not the default `physics-sidecar`. */
  readonly VITE_STDB_DB?: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
