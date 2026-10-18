/**
 * The mission the live run is flying, drawn before it starts.
 *
 * A ground station shows the plan as well as the flight: the numbered waypoints,
 * the route between them, and the spherical keep-out zones. Framing the mission
 * up front is also what makes the camera work, because a live trajectory is a
 * single point on the first frame.
 *
 * Keep-out zones are tinted by the live clearance, the same way the replay tints
 * them: amber while the vehicle is clear, hazard rose when it is inside. The
 * clearance is read from the recorder rather than recomputed from the session, so
 * the tint matches the trail.
 */

import { useEffect, useMemo, useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import type { LiveConfig } from '@/live/types'
import { nedToScene } from '@/utils/interpolation'
import {
  CLEARANCE_CAUTION_M,
  CLEARANCE_COLORS,
  clearanceBand,
  obstacleClearance,
} from '@/utils/clearance'
import { emptyFrame, LiveBuffer } from '@/live/stream'
import { labelTexture } from './labels'

/** Reused between frames so the tint pass allocates nothing. */
const scratch = emptyFrame()

/** Planned route: start, then each waypoint, as a thin dashed line. */
function RouteLine({ cfg }: { cfg: LiveConfig }) {
  const line = useMemo(() => {
    const points = [cfg.start_position_ned, ...cfg.waypoints.map((w) => w.position)]
    const positions = new Float32Array(points.length * 3)
    points.forEach((ned, i) => {
      const [x, y, z] = nedToScene(ned)
      positions[i * 3] = x
      positions[i * 3 + 1] = y
      positions[i * 3 + 2] = z
    })
    const geometry = new THREE.BufferGeometry()
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    const material = new THREE.LineDashedMaterial({
      color: '#26b8d9',
      transparent: true,
      opacity: 0.45,
      dashSize: 1.4,
      gapSize: 1.1,
    })
    const obj = new THREE.Line(geometry, material)
    obj.computeLineDistances()
    obj.frustumCulled = false
    return obj
  }, [cfg])

  useEffect(
    () => () => {
      line.geometry.dispose()
      ;(line.material as THREE.Material).dispose()
    },
    [line],
  )

  if (cfg.waypoints.length === 0) return null
  return <primitive object={line} />
}

/** Waypoints as numbered markers with a ground ring at their own altitude. */
function Waypoints({ cfg }: { cfg: LiveConfig }) {
  const markers = useMemo(
    () =>
      cfg.waypoints.map((w, i) => ({
        key: i,
        scene: nedToScene(w.position),
        label: `WP${i + 1}`,
      })),
    [cfg],
  )

  const textures = useMemo(
    () =>
      markers.map((m) =>
        labelTexture([
          { text: m.label, size: 34, color: '#dbe4f0' },
          { text: 'HOLD', size: 18, color: '#8b9bb4' },
        ]),
      ),
    [markers],
  )

  useEffect(
    () => () => {
      for (const tex of textures) tex.dispose()
    },
    [textures],
  )

  return (
    <group>
      {markers.map((m, i) => (
        <group key={m.key} position={m.scene}>
          <mesh>
            <sphereGeometry args={[0.32, 20, 14]} />
            <meshBasicMaterial color="#dbe4f0" transparent opacity={0.9} />
          </mesh>
          {/* Ground ring, dropped to y = 0 so the waypoint's own altitude reads
              as the distance between the marker and its footprint. */}
          <mesh position={[0, -m.scene[1] + 0.03, 0]} rotation={[-Math.PI / 2, 0, 0]}>
            <ringGeometry args={[0.42, 0.56, 28]} />
            <meshBasicMaterial color="#26b8d9" transparent opacity={0.6} side={THREE.DoubleSide} />
          </mesh>
          {/* Altitude stem: the visual link between the marker and its footprint. */}
          {m.scene[1] > 0.2 && (
            <mesh position={[0, -m.scene[1] / 2, 0]}>
              <cylinderGeometry args={[0.015, 0.015, m.scene[1], 6]} />
              <meshBasicMaterial color="#26b8d9" transparent opacity={0.22} depthWrite={false} />
            </mesh>
          )}
          <sprite position={[0, 1.15, 0]} scale={[1.9, 0.95, 1]}>
            <spriteMaterial map={textures[i]} transparent depthWrite={false} />
          </sprite>
        </group>
      ))}
    </group>
  )
}

/** Spherical keep-out zones, tinted by the live clearance. */
function KeepOutZones({ cfg, buffer }: { cfg: LiveConfig; buffer: LiveBuffer }) {
  const obstacles = cfg.obstacles
  const fillRefs = useRef<(THREE.Mesh | null)[]>([])
  const wireRefs = useRef<(THREE.Mesh | null)[]>([])

  const labels = useMemo(
    () =>
      obstacles.map((o) =>
        labelTexture([
          { text: o.id, size: 30, color: '#e8edf5' },
          { text: `R ${o.radius_m.toFixed(1)} m`, size: 22, color: '#9aa9bf' },
        ]),
      ),
    [obstacles],
  )

  useEffect(
    () => () => {
      for (const tex of labels) tex.dispose()
    },
    [labels],
  )

  useFrame(() => {
    if (obstacles.length === 0) return
    // Read the pose the scene is drawing, so the tint and the trail agree.
    if (!buffer.poseAt(buffer.clock, scratch)) return
    for (let i = 0; i < obstacles.length; i += 1) {
      const obstacle = obstacles[i]
      if (obstacle === undefined) continue
      const band = clearanceBand(
        obstacleClearance(scratch.truthPosition, obstacle),
        CLEARANCE_CAUTION_M,
      )
      const hex =
        band === 'breach'
          ? CLEARANCE_COLORS.breach
          : band === 'caution'
            ? CLEARANCE_COLORS.caution
            : CLEARANCE_COLORS.clear
      const fillMat = fillRefs.current[i]?.material as THREE.MeshBasicMaterial | undefined
      const wireMat = wireRefs.current[i]?.material as THREE.MeshBasicMaterial | undefined
      fillMat?.color.set(hex)
      wireMat?.color.set(hex)
      if (wireMat) wireMat.opacity = band === 'clear' ? 0.3 : 0.5
    }
  })

  if (obstacles.length === 0) return null

  return (
    <group>
      {obstacles.map((obstacle, i) => {
        const [x, y, z] = nedToScene(obstacle.center_ned)
        const stem = Math.max(y, 0.001)
        return (
          <group key={obstacle.id} position={[x, y, z]}>
            <mesh position={[0, -y + 0.02, 0]} rotation={[-Math.PI / 2, 0, 0]}>
              <ringGeometry args={[obstacle.radius_m * 0.96, obstacle.radius_m, 48]} />
              <meshBasicMaterial
                color={CLEARANCE_COLORS.clear}
                transparent
                opacity={0.35}
                side={THREE.DoubleSide}
                depthWrite={false}
              />
            </mesh>
            {y > 0.1 && (
              <mesh position={[0, -stem / 2, 0]}>
                <cylinderGeometry args={[0.03, 0.03, stem, 8]} />
                <meshBasicMaterial
                  color={CLEARANCE_COLORS.clear}
                  transparent
                  opacity={0.3}
                  depthWrite={false}
                />
              </mesh>
            )}
            <mesh ref={(el) => { fillRefs.current[i] = el }}>
              <sphereGeometry args={[obstacle.radius_m, 32, 24]} />
              <meshBasicMaterial
                color={CLEARANCE_COLORS.clear}
                transparent
                opacity={0.08}
                depthWrite={false}
                side={THREE.DoubleSide}
              />
            </mesh>
            <mesh ref={(el) => { wireRefs.current[i] = el }}>
              <sphereGeometry args={[obstacle.radius_m, 20, 14]} />
              <meshBasicMaterial
                color={CLEARANCE_COLORS.clear}
                wireframe
                transparent
                opacity={0.3}
                depthWrite={false}
              />
            </mesh>
            <sprite position={[0, obstacle.radius_m + 1.6, 0]} scale={[3.0, 1.5, 1]}>
              <spriteMaterial map={labels[i]} transparent depthWrite={false} />
            </sprite>
          </group>
        )
      })}
    </group>
  )
}

/** The mission: route, waypoints, and keep-out zones. */
export function LiveField({ cfg, buffer }: { cfg: LiveConfig; buffer: LiveBuffer }) {
  return (
    <group>
      <RouteLine cfg={cfg} />
      <Waypoints cfg={cfg} />
      <KeepOutZones cfg={cfg} buffer={buffer} />
    </group>
  )
}
