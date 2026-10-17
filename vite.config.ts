import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import path from 'path'

// The web app is the repository root; GitHub Pages serves it at the repo
// Pages root (https://<user>.github.io/<repo>/), so the base is '/'.
export default defineConfig({
  plugins: [react()],
  base: '/',
  resolve: {
    alias: {
      '@': path.resolve(__dirname, './src'),
    },
  },
  build: {
    outDir: 'dist',
    chunkSizeWarningLimit: 1200,
  },
})
