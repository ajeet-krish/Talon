import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import path from 'path'

// Talon FlightLab is a GitHub Pages *project* site, served from a subpath:
// https://<owner>.github.io/Talon/. Every asset URL therefore has to carry
// that prefix, so the default base is '/Talon/' and the built index.html
// references /Talon/assets/... rather than /assets/...
//
// The deploy workflow derives the same value from the repository name and
// passes it in as VITE_BASE, so renaming the repository keeps the site
// working without editing this file. Override locally when you need a
// root-served session, for example: VITE_BASE=/ npm run dev
const base = process.env.VITE_BASE ?? '/Talon/'

export default defineConfig({
  plugins: [react()],
  base,
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
