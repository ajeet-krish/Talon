import { useEffect, useMemo, useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import type { FlightData, Obstacle } from '@/types/flight'
import { nedToScene, sampleVec3 } from '@/utils/interpolation'
import {
  CLEARANCE_CAUTION_M,
  CLEARANCE_COLORS,
  clearanceBand,
  obstacleClearance,
} from '@/utils/clearance'
import { useFlightStore } from '@/store/useFlightStore'

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

/**
 * Spherical keep-out zones from schema 1.1 as translucent hazard volumes: a
 * low-opacity fill, a brighter wireframe, a thin drop stem to the ground, and
 * a two-line id + radius label. The tint is data-driven, shifting amber ->
 * caution -> hazard rose as the truth trajectory closes on the surface, so
 * the scene reports real clearance rather than a static sticker.
 *
 * Toggle-controlled; renders nothing when the scenario ships no obstacles.
 */
export function ObstacleField({ data }: { data: FlightData }) {
  const obstacles = data.obstacles
  const enabled = useFlightStore((s) => s.toggles.obstacles)

  const fillRefs = useRef<(THREE.Mesh | null)[]>([])
  const wireRefs = useRef<(THREE.Mesh | null)[]>([])

  const labels = useMemo(
    () => obstacles.map((o) => makeObstacleLabel(o)),
    [obstacles],
  )

  useEffect(() => {
    return () => {
      for (const tex of labels) tex.dispose()
    }
  }, [labels])

  useFrame(() => {
    if (!enabled || obstacles.length === 0) return
    const t = useFlightStore.getState().currentTime
    const posNed = sampleVec3(t, data.time_s, data.truth_position)
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
      if (wireMat) wireMat.opacity = band === 'clear' ? 0.3 : 0.5
    }
  })

  if (obstacles.length === 0) return null

  return (
    <group visible={enabled}>
      {obstacles.map((obstacle, i) => {
        const [x, y, z] = nedToScene(obstacle.center_ned)
        const stemHeight = Math.max(y, 0.001)
        return (
          <group key={obstacle.id} position={[x, y, z]}>
            {/* Ground footprint: vertical projection of the keep-out column. */}
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
            {/* Thin drop stem anchoring the volume to the ground plane. */}
            {y > 0.1 && (
              <mesh position={[0, -stemHeight / 2, 0]}>
                <cylinderGeometry args={[0.03, 0.03, stemHeight, 8]} />
                <meshBasicMaterial
                  color={CLEARANCE_COLORS.clear}
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
            <sprite position={[0, obstacle.radius_m + 1.6, 0]} scale={[3.0, 1.12, 1]}>
              <spriteMaterial map={labels[i]} transparent depthWrite={false} />
            </sprite>
          </group>
        )
      })}
    </group>
  )
}
