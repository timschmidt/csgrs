import wasm from "vite-plugin-wasm";
import topLevelAwait from "vite-plugin-top-level-await";

// https://nuxt.com/docs/api/configuration/nuxt-config
export default defineNuxtConfig({
  compatibilityDate: '2025-07-15',
  devtools: { enabled: true },
  
  // Enable WebAssembly support
  vite: {
    server: {
      fs: {
        allow: ['../../bin'] // Adjust the path to your WASM binary directory
      }
    },
    plugins: [
    wasm(),
    topLevelAwait()
  ]
  },

  // For static generation
  nitro: {
    experimental: {
      wasm: true
    }
  }
})
