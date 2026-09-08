import { defineConfig } from 'vite';

export default defineConfig({
  server: { port: 5173, strictPort: false },
  build: { target: 'es2022', sourcemap: true },
  // physics.wasm lives in public/ and is fetched at runtime, so Vite serves it
  // untouched in dev and copies it verbatim into dist/.
});
