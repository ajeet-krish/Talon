import { useEffect, useMemo, useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import type { FlightData } from '@/types/flight'
import { sampleVec3 } from '@/utils/interpolation'
import { useFlightStore } from '@/store/useFlightStore'

const PARTICLE_COUNT = 350
const ADVECTION_WINDOW_S = 8

/**
 * Wind field as drifting particles. Each particle seeds at a fixed point
 * in the flight volume and advects along the true wind vector (sampled
 * from wind_ned at the current time), wrapping every ADVECTION_WINDOW_S
 * seconds. Motion is frozen whenever playback is paused, so the field is
 * data-driven ambient motion, not decoration. Renders nothing when the
 * scenario has effectively no wind (gps_outage, emergency).
 */
export function WindParticles({ data }: { data: FlightData }) {
  const pointsRef = useRef<THREE.Points>(null)

  const hasWind = useMemo(() => {
    let max = 0
    for (const w of data.wind_ned) {
      max = Math.max(max, Math.hypot(w[0], w[1], w[2]))
      if (max > 0.5) return true
    }
    return max > 0.5
  }, [data.wind_ned])

  const { geometry, seeds, offsets } = useMemo(() => {
    const seeds = new Float32Array(PARTICLE_COUNT * 3)
    const offsets = new Float32Array(PARTICLE_COUNT)
    // Seed the box over truth bounds, padded so particles flow through view.
    let minN = Infinity
    let maxN = -Infinity
    let minE = Infinity
    let maxE = -Infinity
    let minU = Infinity
    let maxU = -Infinity
    for (const [n, e, d] of data.truth_position) {
      minN = Math.min(minN, n)
      maxN = Math.max(maxN, n)
      minE = Math.min(minE, e)
      maxE = Math.max(maxE, e)
      minU = Math.min(minU, -d)
      maxU = Math.max(maxU, -d)
    }
    const pad = 12
    minN -= pad
    maxN += pad
    minE -= pad
    maxE += pad
    minU = Math.max(0, minU - 2)
    maxU += pad
    // Deterministic pseudo-random seeding so reloads look identical.
    let s = 20261012
    const rand = () => {
      s = (s * 1664525 + 1013904223) >>> 0
      return s / 4294967296
    }
    for (let i = 0; i < PARTICLE_COUNT; i++) {
      seeds[i * 3] = minN + rand() * (maxN - minN)
      seeds[i * 3 + 1] = minU + rand() * (maxU - minU)
      seeds[i * 3 + 2] = minE + rand() * (maxE - minE)
      offsets[i] = rand() * ADVECTION_WINDOW_S
    }
    const positions = new Float32Array(PARTICLE_COUNT * 3)
    positions.set(seeds)
    const geometry = new THREE.BufferGeometry()
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    return { geometry, seeds, offsets }
  }, [data.truth_position])

  useEffect(() => {
    return () => geometry.dispose()
  }, [geometry])

  useFrame(() => {
    const { currentTime, toggles } = useFlightStore.getState()
    const active = toggles.wind && hasWind
    if (pointsRef.current) pointsRef.current.visible = active
    if (!active) return
    const windNed = sampleVec3(currentTime, data.time_s, data.wind_ned)
    // NED is [north, east, down]; the scene is [north, up, east].
    const [wn, we, wd] = windNed
    const wUp = -wd
    const pos = geometry.getAttribute('position') as THREE.BufferAttribute
    const arr = pos.array as Float32Array
    for (let i = 0; i < PARTICLE_COUNT; i++) {
      const flow = (currentTime + (offsets[i] as number)) % ADVECTION_WINDOW_S
      arr[i * 3] = (seeds[i * 3] as number) + wn * flow
      arr[i * 3 + 1] = (seeds[i * 3 + 1] as number) + wUp * flow
      arr[i * 3 + 2] = (seeds[i * 3 + 2] as number) + we * flow
    }
    pos.needsUpdate = true
  })

  if (!hasWind) return null

  return (
    <points ref={pointsRef} geometry={geometry}>
      <pointsMaterial
        color="#26b8d9"
        size={0.16}
        sizeAttenuation
        transparent
        opacity={0.5}
        depthWrite={false}
      />
    </points>
  )
}
