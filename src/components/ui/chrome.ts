/**
 * Shared chrome styling for the mission page's two framed surfaces (FlightLab
 * Phase 8d-D): the 2D planning chart and the 3D course preview sit side by
 * side and must read as one instrument, so their corner controls carry one
 * class string. Pure string constants: no React, no DOM.
 */

/**
 * A surface's corner control (ZOOM IN, ZOOM OUT, FIT): the toolbar's quiet
 * mono style on a panel backing, with a single signal-cyan focus ring.
 */
export const CHROME_BUTTON =
  'border border-grid bg-panel/85 px-3 py-2 font-mono text-[10px] tracking-[0.15em] text-ink-dim transition-colors hover:bg-panel-3 hover:text-ink focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-signal disabled:opacity-40 disabled:hover:bg-transparent disabled:hover:text-ink-dim'
