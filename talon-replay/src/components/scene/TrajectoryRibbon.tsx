import { useEffect, useMemo } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import type { FlightData } from '@/types/flight'
import { nedToScene, phaseAtTime } from '@/utils/interpolation'
import { PHASE_COLORS } from '@/utils/colors'
import { useFlightStore } from '@/store/useFlightStore'

/**
 * Truth trajectory as a single colored line. Geometry is built once per
 * scenario: every sample is a vertex in scene space, pre-colored by the
 * Wong-palette color of the flight phase at that instant. Playback reveals
 * the line by advancing the draw range to the current sample, so scrubbing
 * draws and erases the flight path under the vehicle.
 */
export function TrajectoryRibbon({ data }: { data: FlightData }) {
  const line = useMemo(() => {
    const n = data.time_s.length
    const positions = new Float32Array(n * 3)
    const colors = new Float32Array(n * 3)
    const color = new THREE.Color()
    let phase = phaseAtTime(data.phase_timeline, 0)
    for (let i = 0; i < n; i++) {
      const ned = data.truth_position[i] ?? [0, 0, 0]
      const [x, y, z] = nedToScene(ned)
      positions[i * 3] = x
      positions[i * 3 + 1] = y
      positions[i * 3 + 2] = z
      phase = phaseAtTime(data.phase_timeline, data.time_s[i] ?? 0)
      color.set(PHASE_COLORS[phase])
      colors[i * 3] = color.r
      colors[i * 3 + 1] = color.g
      colors[i * 3 + 2] = color.b
    }
    const geometry = new THREE.BufferGeometry()
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3))
    geometry.setDrawRange(0, 2)
    const material = new THREE.LineBasicMaterial({
      vertexColors: true,
      transparent: true,
      opacity: 0.95,
    })
    const obj = new THREE.Line(geometry, material)
    obj.frustumCulled = false
    return obj
  }, [data])

  useEffect(() => {
    return () => {
      line.geometry.dispose()
      ;(line.material as THREE.Material).dispose()
    }
  }, [line])

  useFrame(() => {
    const { currentTime, toggles } = useFlightStore.getState()
    line.visible = toggles.trail
    const dt = data.meta.dt
    const idx = Math.min(
      Math.max(Math.floor(currentTime / dt) + 1, 2),
      data.time_s.length,
    )
    line.geometry.setDrawRange(0, idx)
  })

  return <primitive object={line} />
}
