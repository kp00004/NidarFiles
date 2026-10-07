/** @type {import('tailwindcss').Config} */
export default {
  content: ["./index.html", "./src/**/*.{ts,tsx}"],
  theme: {
    extend: {
      colors: {
        // Matches the dark operator-panel palette from the previous
        // prototype (gcs/frontend/index.html, pre-React) so the visual
        // language doesn't reset with the framework migration.
        bg: "#0b0f14",
        panel: "#131a22",
        border: "#26313d",
        text: "#e6edf3",
        dim: "#8b98a5",
        ok: "#2ea043",
        bad: "#da3633",
        warn: "#d29922",
        accent: "#388bfd",
      },
    },
  },
  plugins: [],
};
