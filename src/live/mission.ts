/**
 * Waypoint mission manager (TypeScript port of talon/mission.py).
 *
 * Owns the waypoint list, the active-index bookkeeping, and the acceptance
 * radius logic. Guidance is a pure function of (position, velocity, leg); it
 * never advances waypoints itself. Positions are NED (m); acceptance and
 * distance are horizontal-plane (North-East) only.
 */

import type { Waypoint } from './types'
import type { Vec3 } from './math'

/** North-East plane distance between two NED positions (m). */
function horizontalDistance(a: Vec3, b: Vec3): number {
  const dn = a[0] - b[0]
  const de = a[1] - b[1]
  return Math.sqrt(dn * dn + de * de)
}

export class WaypointMission {
  private readonly _waypoints: Waypoint[]
  private _activeIndex = 0
  private _complete = false
  private _startPosition: Vec3 | null = null

  constructor(waypoints: Waypoint[]) {
    if (waypoints.length === 0) {
      throw new Error('waypoints must be non-empty')
    }
    this._waypoints = waypoints.slice()
  }

  get activeIndex(): number {
    return this._activeIndex
  }

  get complete(): boolean {
    return this._complete
  }

  get waypoints(): Waypoint[] {
    return this._waypoints.slice()
  }

  activeWaypoint(): Waypoint {
    return this._waypoints[this._activeIndex] as Waypoint
  }

  /** Record the mission start position (leg origin of the first leg). */
  setStart(position: Vec3): void {
    this._startPosition = [position[0], position[1], position[2]]
  }

  /**
   * Start point of the active leg (NED, m): waypoint[i-1] once past the
   * first waypoint, else the recorded mission start, else the current
   * position so guidance always has a valid leg origin.
   */
  previousPoint(currentPosition: Vec3): Vec3 {
    if (this._activeIndex > 0) {
      const w = this._waypoints[this._activeIndex - 1] as Waypoint
      return [w.position[0], w.position[1], w.position[2]]
    }
    if (this._startPosition !== null) {
      return [
        this._startPosition[0],
        this._startPosition[1],
        this._startPosition[2],
      ]
    }
    return [currentPosition[0], currentPosition[1], currentPosition[2]]
  }

  /**
   * Advance the active waypoint while within its radius. Records the
   * mission start on the first call if not set. Returns true once the
   * mission is complete (latched).
   */
  update(position: Vec3): boolean {
    if (!position.every(Number.isFinite)) {
      throw new Error(`position must be finite, got ${String(position)}`)
    }
    if (this._startPosition === null) {
      this._startPosition = [position[0], position[1], position[2]]
    }

    while (!this._complete) {
      const active = this._waypoints[this._activeIndex] as Waypoint
      if (horizontalDistance(position, active.position) >= active.radius) {
        break
      }
      if (this._activeIndex + 1 >= this._waypoints.length) {
        this._complete = true
      } else {
        this._activeIndex += 1
      }
    }
    return this._complete
  }

  /** Horizontal distance still to fly (m), including remaining legs. */
  distanceRemaining(position: Vec3): number {
    let total = horizontalDistance(position, this.activeWaypoint().position)
    for (let i = this._activeIndex; i < this._waypoints.length - 1; i += 1) {
      const a = this._waypoints[i] as Waypoint
      const b = this._waypoints[i + 1] as Waypoint
      total += horizontalDistance(a.position, b.position)
    }
    return total
  }
}
