/**
 * Playback pacing for the live run.
 *
 * The native loop is far faster than real time: a 120 s mission is a few tens of
 * milliseconds of work. That is a feature for batch runs and a problem for a
 * ground station, where the point is to watch the vehicle fly and read the
 * numbers as they change. So the run is paced against the wall clock instead of
 * being run to completion: this class answers one question per frame, "what sim
 * time should we be at now?", and the run view advances the native session by
 * however many plant steps that implies.
 *
 * Pacing is a display concern and lives entirely on this side of the boundary.
 * The core has no idea it is being paced, which is what keeps a paced run and a
 * batch run the same flight.
 *
 * `Infinity` means "fastest": the target is the end of the run, so the frame
 * budget is the only limit. Changing rate mid-run re-bases on the current sim
 * time, so it never jumps the vehicle forward or backward, and pausing simply
 * stops accumulating wall time.
 */

/** Selectable playback rates. `Infinity` runs as fast as the frame allows. */
export const RATES = [1, 4, 16, Infinity] as const

export type Rate = (typeof RATES)[number]

/**
 * Largest wall-time step a frame may contribute (ms).
 *
 * A hidden tab stops firing animation frames, so the first frame back carries a
 * delta of however long the tab was away. Absorbing that would teleport the
 * vehicle, so it is clamped. The value is generous enough to cover a slow frame
 * on a loaded machine without letting the run fall permanently behind.
 */
export const MAX_FRAME_MS = 250

/** Label for a rate, for the control strip. */
export function rateLabel(rate: number): string {
  if (!Number.isFinite(rate)) return 'MAX'
  return `${rate}\u00d7`
}

export class Pacer {
  private readonly durationS: number
  private rate: number
  /** Sim time the current wall-clock segment started from. */
  private baseS: number
  /** Wall milliseconds accumulated in the current segment. */
  private wallMs = 0
  private running = false

  constructor(durationS: number, rate: number, startS = 0) {
    this.durationS = Math.max(durationS, 0)
    this.rate = rate
    this.baseS = startS
  }

  get isRunning(): boolean {
    return this.running
  }

  get playbackRate(): number {
    return this.rate
  }

  /** Begin (or resume) accumulating wall time from `simTime`. */
  start(simTime: number): void {
    this.baseS = simTime
    this.wallMs = 0
    this.running = true
  }

  /** Stop accumulating wall time; the target holds where it is. */
  pause(simTime: number): void {
    this.baseS = simTime
    this.wallMs = 0
    this.running = false
  }

  /** Back to the beginning, stopped. */
  reset(): void {
    this.baseS = 0
    this.wallMs = 0
    this.running = false
  }

  /**
   * Change the rate without moving the vehicle.
   *
   * Re-bases on the current sim time, so a run at 16x dropping to 1x continues
   * from where it is rather than snapping to wherever the old rate would have
   * put it.
   */
  setRate(rate: number, simTime: number): void {
    this.rate = rate
    this.baseS = simTime
    this.wallMs = 0
  }

  /** Account for one frame's wall time (ms). Ignored while paused. */
  tick(deltaMs: number): void {
    if (!this.running) return
    this.wallMs += Math.min(Math.max(deltaMs, 0), MAX_FRAME_MS)
  }

  /** Sim time the run should be at now (s), clamped to the run duration. */
  targetTime(): number {
    if (!Number.isFinite(this.rate)) return this.durationS
    const ahead = (this.wallMs / 1000) * this.rate
    return Math.min(this.baseS + ahead, this.durationS)
  }

  /** Whether the paced run has reached its end. */
  isComplete(): boolean {
    return this.targetTime() >= this.durationS
  }
}
