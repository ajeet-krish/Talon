import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import path from 'path'

// GitHub Pages serves the app under /talon-replay/.
export default defineConfig({
  plugins: [react()],
  base: '/talon-replay/',
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
