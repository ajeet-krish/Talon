import { useEffect, useMemo, useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import type { Obstacle, Vec3 } from '@/types/flight'
import { nedToScene, sampleVec3 } from '@/utils/interpolation'
import {
  CLEARANCE_CAUTION_M,
  CLEARANCE_COLORS,
  clearanceBand,
  obstacleClearance,
} from '@/utils/clearance'
import { useFlightStore } from '@/store/useFlightStore'

/**
 * The recorded drone track the live clearance cue samples, narrowed from
 * `FlightData` to the two series it reads. Present in the replay; absent in
 * the mission preview, which has no telemetry yet, so the cue runs only
 * where there is a flight to report on.
 */
export interface DroneTrack {
  time_s: readonly number[]
  truth_position: readonly Vec3[]
}

/**
 * Zone tint with no live cue: the editor's warn amber, the same `#e0a458`
 * the 2D chart draws its keep-out circles in, so a zone in the mission
 * preview reads as a keep-out rather than as a clearance reading.
 */
const STATIC_ZONE_COLOR = '#e0a458'

/** Two-line canvas label (id + radius) so labels stay offline-safe. */
function makeObstacleLabel(obstacle: Obstacle): THREE.CanvasTexture {
  const canvas = document.createElement('canvas')
  canvas.width = 256
  canvas.height = 96
  const ctx = canvas.getContext('2d')
  if (ctx) {
    ctx.font = '600 34px "IBM Plex Mono", monospace'
    ctx.fillStyle = '#e8edf5'
    ctx.textAlign = 'center'
    ctx.textBaseline = 'middle'
    ctx.fillText(obstacle.id, 128, 32)
    ctx.font = '500 26px "IBM Plex Mono", monospace'
    ctx.fillStyle = '#9aa9bf'
    ctx.fillText(`R ${obstacle.radius_m.toFixed(1)} m`, 128, 70)
  }
  const tex = new THREE.CanvasTexture(canvas)
  tex.colorSpace = THREE.SRGBColorSpace
  return tex
}

export interface ObstacleFieldProps {
  /** The keep-out zones to draw, in NED. */
  obstacles: Obstacle[]
  /**
   * Where the vehicle is right now: when present, each zone is tinted by
   * its live clearance every frame (the replay's data-driven readout). The
   * mission preview passes nothing, so its zones keep the static amber the
   * chart uses. Input is a slice rather than the whole `FlightData`.
   */
  droneTrack?: DroneTrack
  /**
   * Index of the zone the editor points at: its shell and footprint read
   * brighter, the way the chart bolds the selected circle. The replay
   * passes nothing.
   */
  highlight?: number
  /**
   * Layer visibility. The replay passes its display toggle; the preview
   * leaves it at the default.
   */
  visible?: boolean
}

/**
 * Spherical keep-out zones from schema 1.1 as translucent hazard volumes: a
 * low-opacity fill, a brighter wireframe, a thin drop stem to the ground,
 * and a two-line id + radius label. With a drone track the tint is
 * data-driven, shifting amber -> caution -> hazard rose as the truth
 * trajectory closes on the surface, so the scene reports real clearance
 * rather than a static sticker; without one the zones hold the chart's
 * amber.
 *
 * Renders nothing when the mission ships no zones.
 */
export function ObstacleField({
  obstacles,
  droneTrack,
  highlight,
  visible = true,
}: ObstacleFieldProps) {
  const live = droneTrack !== undefined

  const fillRefs = useRef<(THREE.Mesh | null)[]>([])
  const wireRefs = useRef<(THREE.Mesh | null)[]>([])

  /**
   * Texture cache keyed on what the label text carries: the id and the
   * radius at label precision. A zone MOVE leaves both untouched, so a
   * live drag reuses the canvases instead of redrawing them per pointer
   * move; a resize regenerates, because the radius on the label changed.
   */
  const labelKey = obstacles.map((o) => `${o.id}:${o.radius_m.toFixed(1)}`).join(' ')
  const labels = useMemo(() => obstacles.map((o) => makeObstacleLabel(o)), [labelKey]) // labelKey covers every label

  useEffect(() => {
    return () => {
      for (const tex of labels) tex.dispose()
    }
  }, [labels])

  useFrame(() => {
    if (!visible || obstacles.length === 0) return
    const track = droneTrack
    if (track === undefined) return // no telemetry, no cue: the preview's zones stay amber
    const t = useFlightStore.getState().currentTime
    const posNed = sampleVec3(t, track.time_s, track.truth_position)
    for (let i = 0; i < obstacles.length; i++) {
      const obstacle = obstacles[i]
      if (!obstacle) continue
      const band = clearanceBand(
        obstacleClearance(posNed, obstacle),
        CLEARANCE_CAUTION_M,
      )
      const hex =
        band === 'breach'
          ? CLEARANCE_COLORS.breach
          : band === 'caution'
            ? CLEARANCE_COLORS.caution
            : CLEARANCE_COLORS.clear
      const fill = fillRefs.current[i]
      const wire = wireRefs.current[i]
      const fillMat = fill?.material as THREE.MeshBasicMaterial | undefined
      const wireMat = wire?.material as THREE.MeshBasicMaterial | undefined
      fillMat?.color.set(hex)
      wireMat?.color.set(hex)
      if (wireMat) wireMat.opacity = i === highlight ? 0.55 : band === 'clear' ? 0.3 : 0.5
    }
  })

  if (obstacles.length === 0) return null

  /** Chart amber without a live cue; the clearance palette with one. */
  const zoneColor = live ? CLEARANCE_COLORS.clear : STATIC_ZONE_COLOR

  return (
    <group visible={visible}>
      {obstacles.map((obstacle, i) => {
        const [x, y, z] = nedToScene(obstacle.center_ned)
        const stemHeight = Math.max(y, 0.001)
        const hot = highlight === i
        return (
          <group key={obstacle.id} position={[x, y, z]}>
            {/* Ground footprint: vertical projection of the keep-out column. */}
            <mesh position={[0, -y + 0.02, 0]} rotation={[-Math.PI / 2, 0, 0]}>
              <ringGeometry args={[obstacle.radius_m * 0.96, obstacle.radius_m, 48]} />
              <meshBasicMaterial
                color={zoneColor}
                transparent
                opacity={hot ? 0.75 : 0.35}
                side={THREE.DoubleSide}
                depthWrite={false}
              />
            </mesh>
            {/* Thin drop stem anchoring the volume to the ground plane. */}
            {y > 0.1 && (
              <mesh position={[0, -stemHeight / 2, 0]}>
                <cylinderGeometry args={[0.03, 0.03, stemHeight, 8]} />
                <meshBasicMaterial
                  color={zoneColor}
                  transparent
                  opacity={0.3}
                  depthWrite={false}
                />
              </mesh>
            )}
            {/* Translucent keep-out volume + wireframe shell. */}
            <mesh ref={(el) => { fillRefs.current[i] = el }}>
              <sphereGeometry args={[obstacle.radius_m, 32, 24]} />
              <meshBasicMaterial
                color={zoneColor}
                transparent
                opacity={hot ? 0.14 : 0.08}
                depthWrite={false}
                side={THREE.DoubleSide}
              />
            </mesh>
            <mesh ref={(el) => { wireRefs.current[i] = el }}>
              <sphereGeometry args={[obstacle.radius_m, 20, 14]} />
              <meshBasicMaterial
                color={zoneColor}
                wireframe
                transparent
                opacity={hot ? 0.55 : 0.3}
                depthWrite={false}
              />
            </mesh>
            <sprite position={[0, obstacle.radius_m + 1.6, 0]} scale={[3.0, 1.12, 1]}>
              <spriteMaterial map={labels[i]} transparent depthWrite={false} />
            </sprite>
          </group>
        )
      })}
    </group>
  )
}
