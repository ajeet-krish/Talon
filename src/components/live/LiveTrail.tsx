/**
 * The live trajectory, revealed as the vehicle flies it.
 *
 * The geometry wraps the recorder's trail array directly: `buffer.trail` is a
 * `Float32Array` that `append` already writes in scene space, so the attribute
 * and the recorder share one buffer and no copy happens per frame. The line is
 * revealed by advancing the draw range to the number of samples recorded, the
 * same trick the replay ribbon uses.
 *
 * Vertex colours come from the phase each sample was recorded in, so the trail
 * accumulates the colour of the flight phases and the map shows the mission's
 * shape as well as its path.
 */

import { useEffect, useMemo, useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import { PHASE_COLORS } from '@/utils/colors'
import { PHASE_ORDER, type LiveBuffer } from '@/live/stream'

/** Palette lookup, resolved once: index -> linear RGB. */
const PALETTE: [number, number, number][] = PHASE_ORDER.map((phase) => {
  const c = new THREE.Color(PHASE_COLORS[phase])
  return [c.r, c.g, c.b]
})

export function LiveTrail({ buffer }: { buffer: LiveBuffer }) {
  /** Samples already coloured, so only the new tail is written each frame. */
  const coloured = useRef(0)

  const { geometry, material, line } = useMemo(() => {
    const positions = new THREE.BufferAttribute(buffer.trail, 3)
    positions.setUsage(THREE.DynamicDrawUsage)
    const colors = new THREE.BufferAttribute(new Float32Array(buffer.sampleCapacity * 3), 3)
    colors.setUsage(THREE.DynamicDrawUsage)
    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', positions)
    geo.setAttribute('color', colors)
    geo.setDrawRange(0, 0)
    const mat = new THREE.LineBasicMaterial({
      vertexColors: true,
      transparent: true,
      opacity: 0.95,
    })
    const obj = new THREE.Line(geo, mat)
    obj.frustumCulled = false
    return { geometry: geo, material: mat, line: obj }
  }, [buffer])

  useEffect(() => {
    coloured.current = 0
    return () => {
      geometry.dispose()
      material.dispose()
    }
  }, [geometry, material])

  useFrame(() => {
    const count = buffer.count
    if (count === coloured.current) return
    const colors = geometry.getAttribute('color') as THREE.BufferAttribute
    const arr = colors.array as Float32Array
    for (let i = coloured.current; i < count; i += 1) {
      const rgb = PALETTE[buffer.phaseIndex[i] ?? 0] ?? [1, 1, 1]
      arr[i * 3] = rgb[0]
      arr[i * 3 + 1] = rgb[1]
      arr[i * 3 + 2] = rgb[2]
    }
    coloured.current = count
    const positions = geometry.getAttribute('position') as THREE.BufferAttribute
    positions.needsUpdate = true
    colors.needsUpdate = true
    geometry.setDrawRange(0, count)
  })

  return <primitive object={line} />
}
