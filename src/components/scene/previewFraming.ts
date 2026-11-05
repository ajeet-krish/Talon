/**
 * Mission preview framing (FlightLab Phase 8d-D): the pure maths behind the
 * 3D course preview that sits beside the 2D chart. Scene space is
 * `nedToScene`'s: x = north, y = up, z = east, ground on y = 0. Everything
 * derives from the authored draft only, so the preview works with zero
 * telemetry: no run, no LiveBuffer, no frame loop.
 *
 * The preview is the chart's partner and borrows the chart's framing
 * habits: the same padding (`CHART_PADDING_M`), the same empty field
 * (`CHART_EMPTY_SPAN_M`), and the same round-meter grid ladder
 * (`gridStepM`), so the two surfaces frame one mission the same way.
 * `components/editor/chart.ts` stays the chart's own viewport module; this
 * one answers the three questions the preview's camera and ground grid ask.
 */

import { CHART_EMPTY_SPAN_M, CHART_PADDING_M, gridStepM } from '@/components/editor/chart'
import type { Vec3 } from '@/types/flight'
import type { MissionConfig } from '@/utils/editor'
import { nedToScene } from '@/utils/interpolation'

/** Framed scene-space box, padded, with the sphere that holds it. */
export interface PreviewBounds {
  /** Smallest corner (x, y, z). */
  min: Vec3
  /** Largest corner (x, y, z). */
  max: Vec3
  /** Box centre: the camera's target once clamped above the ground. */
  center: Vec3
  /** Half the box diagonal: every framed point lies within it of the centre. */
  radius: number
}

/** Where the preview camera stands to frame a set of bounds. */
export interface PreviewCameraPose {
  position: Vec3
  /** Orbit centre: the bounds centre, never below the ground plane. */
  target: Vec3
  near: number
  far: number
  /** Wheel and keyboard zoom floor, set on the controls with the pose. */
  minDistance: number
  /** Wheel and keyboard zoom ceiling, set on the controls with the pose. */
  maxDistance: number
}

/** The preview's ground grid: extent, cells, and its world x/z centre. */
export interface PreviewGrid {
  size: number
  divisions: number
  /** Grid centre snapped to a cell line, so lines land on round meters. */
  position: Vec3
}

/** Vertical field of view, matching the replay camera. */
export const PREVIEW_FOV_DEG = 42

/**
 * The replay `CameraRig`'s framing direction, so the preview camera stands
 * where the replay camera would: up, back, and to the north-east.
 */
const VIEW_DIRECTION: Vec3 = [1.25, 0.85, 1.15]
const VIEW_DIRECTION_LENGTH = Math.hypot(1.25, 0.85, 1.15)

/** Smallest panel aspect the pose fits for; narrower boxes use this floor. */
const MIN_PANEL_ASPECT = 0.5

/**
 * The scene-space box the preview frames: home, every waypoint, and every
 * zone counted as its full sphere, padded by the chart's margin. With no
 * route and no zones the box is the chart's 60 m empty field laid flat on
 * the start, so an empty draft reads as a field to plan on rather than a
 * point. The radius is half the box diagonal, which is the sphere a camera
 * fit has to hold.
 */
export function missionSceneBounds(config: MissionConfig): PreviewBounds {
  const [sx, sy, sz] = nedToScene(config.start_position_ned)
  let minX = sx
  let maxX = sx
  let minY = sy
  let maxY = sy
  let minZ = sz
  let maxZ = sz

  const hasGeometry = config.waypoints.length > 0 || config.obstacles.length > 0
  if (!hasGeometry) {
    // The chart's empty field (padding already inside the 60 m), plus
    // vertical room so the framed box is never a flat disc.
    const half = CHART_EMPTY_SPAN_M / 2 - CHART_PADDING_M
    minX -= half
    maxX += half
    minZ -= half
    maxZ += half
    minY -= 10
    maxY += 10
  }

  for (const wp of config.waypoints) {
    const [x, y, z] = nedToScene(wp.position)
    minX = Math.min(minX, x)
    maxX = Math.max(maxX, x)
    minY = Math.min(minY, y)
    maxY = Math.max(maxY, y)
    minZ = Math.min(minZ, z)
    maxZ = Math.max(maxZ, z)
  }
  for (const ob of config.obstacles) {
    const [x, y, z] = nedToScene(ob.center_ned)
    const r = ob.radius_m
    minX = Math.min(minX, x - r)
    maxX = Math.max(maxX, x + r)
    minY = Math.min(minY, y - r)
    maxY = Math.max(maxY, y + r)
    minZ = Math.min(minZ, z - r)
    maxZ = Math.max(maxZ, z + r)
  }

  minX -= CHART_PADDING_M
  maxX += CHART_PADDING_M
  minY -= CHART_PADDING_M
  maxY += CHART_PADDING_M
  minZ -= CHART_PADDING_M
  maxZ += CHART_PADDING_M

  const center: Vec3 = [(minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2]
  const radius = Math.max(Math.hypot(maxX - minX, maxY - minY, maxZ - minZ) / 2, 8)
  return { min: [minX, minY, minZ], max: [maxX, maxY, maxZ], center, radius }
}

/**
 * The camera pose that frames the bounds: along the replay's view direction
 * at the distance where a sphere of `radius` fits the tighter panel axis at
 * 42° vertical fov (8% margin), with zoom clamps and near/far planes scaled
 * from that distance. `aspect` is the panel at the moment of the fit; the
 * 0.5 floor keeps an unmeasured or very narrow panel from flinging the
 * camera out. Distance grows as the panel narrows, so a tall split still
 * shows the whole mission.
 */
export function previewCameraPose(bounds: PreviewBounds, aspect: number): PreviewCameraPose {
  const [cx, cy, cz] = bounds.center
  const target: Vec3 = [cx, Math.max(cy, 0), cz]
  const halfV = ((PREVIEW_FOV_DEG * Math.PI) / 180) / 2
  const safeAspect = Number.isFinite(aspect) && aspect > 0 ? Math.max(aspect, MIN_PANEL_ASPECT) : 1
  const halfH = Math.atan(Math.tan(halfV) * safeAspect)
  const half = Math.min(halfV, halfH)
  const distance = (bounds.radius / Math.sin(half)) * 1.08
  const position: Vec3 = [
    target[0] + (VIEW_DIRECTION[0] / VIEW_DIRECTION_LENGTH) * distance,
    target[1] + (VIEW_DIRECTION[1] / VIEW_DIRECTION_LENGTH) * distance,
    target[2] + (VIEW_DIRECTION[2] / VIEW_DIRECTION_LENGTH) * distance,
  ]
  const minDistance = Math.max(2, distance * 0.08)
  const maxDistance = distance * 4
  const near = Math.max(0.05, minDistance * 0.2)
  // maxDistance + radius <= 5 * distance holds because distance >= radius.
  const far = distance * 5
  return { position, target, near, far, minDistance, maxDistance }
}

/**
 * The ground grid for a set of bounds: the chart's round-meter step over a
 * 10-cell square centred on the mission (snapped to a cell line, with 20%
 * step headroom so the snap always still covers the box). The chart draws
 * about ten lines per frame; this draws ten cells per side to match, and
 * the cells grow with the mission the way the chart's grid does.
 */
export function previewGrid(bounds: PreviewBounds): PreviewGrid {
  const extentX = bounds.max[0] - bounds.min[0]
  const extentZ = bounds.max[2] - bounds.min[2]
  const extent = Math.max(extentX, extentZ, 1)
  // The chart's ladder carries headroom for the centre snap (up to half a
  // cell each way); the extent/9 floor takes over only where the ladder
  // runs out of round steps, so coverage never depends on its ceiling.
  const step = Math.max(gridStepM(extent * 1.2), extent / 9)
  const [cx, , cz] = bounds.center
  return {
    size: step * 10,
    divisions: 10,
    position: [Math.round(cx / step) * step, 0, Math.round(cz / step) * step],
  }
}
