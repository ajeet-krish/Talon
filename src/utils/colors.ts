import type { FlightPhase } from '@/types/flight'

/**
 * Phase colors from the Wong palette, the same set used in the Matplotlib
 * plots shipped with Talon. Reserved exclusively for flight phases, on the
 * trail, the timeline segments, and the HUD chip.
 */
export const PHASE_COLORS: Record<FlightPhase, string> = {
  idle: '#999999',
  arm: '#E69F00',
  takeoff: '#56B4E9',
  cruise: '#0072B2',
  waypoint: '#009E73',
  rtl: '#F0E442',
  land: '#CC79A7',
  emergency: '#D55E00',
}

/** Uppercase label for a phase, for HUD chips and the legend. */
export function phaseLabel(phase: FlightPhase): string {
  return phase.toUpperCase()
}
