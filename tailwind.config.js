/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{js,ts,jsx,tsx}'],
  theme: {
    extend: {
      colors: {
        // Instrument-panel blue-black scale
        panel: '#0b1020',
        'panel-2': '#0e1526',
        'panel-3': '#121a2e',
        grid: '#1a2333',
        'grid-2': '#243044',
        // Signal cyan: active UI, scrubber, focus
        signal: '#26b8d9',
        'signal-dim': '#1a7f96',
        // Cool ink scale
        ink: '#dbe4f0',
        'ink-dim': '#8b9bb4',
        'ink-faint': '#55657f',
      },
      fontFamily: {
        mono: ['"IBM Plex Mono"', 'ui-monospace', 'SFMono-Regular', 'monospace'],
        sans: ['Inter', 'system-ui', '-apple-system', 'sans-serif'],
      },
    },
  },
  plugins: [],
}
