/// <reference types="vitest/config" />
import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Build output lands in dist/, which gcs/backend/app/main.py mounts at
// /ui -- see that file's _FRONTEND_DIR. Base path is relative ("./") so
// the built assets resolve correctly when served from /ui/, not /.
export default defineConfig({
  plugins: [react()],
  base: "./",
  build: {
    outDir: "dist",
    emptyOutDir: true,
  },
  server: {
    // Dev-server convenience only: proxy API calls to the FastAPI
    // backend so `npm run dev` can run on its own port (5173) against a
    // backend started separately, without hardcoding a host in app code.
    proxy: {
      "/api": "http://127.0.0.1:8000",
      "/health": "http://127.0.0.1:8000",
    },
  },
  test: {
    environment: "jsdom",
    globals: true,
    setupFiles: ["./src/test/setup.ts"],
  },
});
