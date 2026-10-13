import { useEffect, useMemo } from 'react'
import * as THREE from 'three'
import type { FlightData } from '@/types/flight'
import { nedToScene } from '@/utils/interpolation'
import { useFlightStore } from '@/store/useFlightStore'


/** Small canvas-rendered label sprite so numbers stay offline-safe. */
function makeLabel(text: string): THREE.CanvasTexture {
  const canvas = document.createElement('canvas')
  canvas.width = 128
  canvas.height = 64
  const ctx = canvas.getContext('2d')
  if (ctx) {
    ctx.font = '600 34px "IBM Plex Mono", monospace'
    ctx.fillStyle = '#dbe4f0'
    ctx.textAlign = 'center'
    ctx.textBaseline = 'middle'
    ctx.fillText(text, 64, 32)
  }
  const tex = new THREE.CanvasTexture(canvas)
  tex.colorSpace = THREE.SRGBColorSpace
  return tex
}

/**
 * Mission waypoints as numbered spheres. Toggle-controlled; the gps_outage
 * scenario has two NLGL waypoints, the others none, so this renders
 * nothing when the mission list is empty.
 */
export function WaypointMarkers({ data }: { data: FlightData }) {
  const waypoints = useMemo(
    () => data.waypoints_ned.map((ned, i) => ({
      key: i,
      scene: nedToScene(ned),
      label: `WP${i + 1}`,
    })),
    [data.waypoints_ned],
  )

  const textures = useMemo(
    () => waypoints.map((w) => makeLabel(w.label)),
    [waypoints],
  )

  useEffect(() => {
    return () => {
      for (const tex of textures) tex.dispose()
    }
  }, [textures])

  const visible = useFlightStore((s) => s.toggles.waypoints)
  if (waypoints.length === 0) return null

  return (
    <group visible={visible}>
      {waypoints.map((w, i) => (
        <group key={w.key} position={[w.scene[0], w.scene[1], w.scene[2]]}>
          <mesh>
            <sphereGeometry args={[0.35, 20, 14]} />
            <meshBasicMaterial color="#dbe4f0" transparent opacity={0.9} />
          </mesh>
          <mesh position={[0, -w.scene[1] + 0.03, 0]} rotation={[-Math.PI / 2, 0, 0]}>
            <ringGeometry args={[0.45, 0.58, 28]} />
            <meshBasicMaterial color="#26b8d9" transparent opacity={0.6} side={2} />
          </mesh>
          <sprite position={[0, 1.2, 0]} scale={[1.7, 0.85, 1]}>
            <spriteMaterial map={textures[i]} transparent depthWrite={false} />
          </sprite>
        </group>
      ))}
    </group>
  )
}
