import * as THREE from 'three'

export interface GroundPlaneProps {
  /** Grid extent (m), square. The replay's 200 m default is unchanged. */
  size?: number
  /** Cells per side. The replay's 40 (5 m cells) is unchanged. */
  divisions?: number
  /** World x/z the grid is centred on; the preview centres it on the mission. */
  position?: [number, number, number]
}

/**
 * Instrument grid on the y = 0 plane, steel lines on the panel blue-black.
 * The drone's soft shadow ellipse is drawn by DroneMarker (it tracks the
 * vehicle); this component is the static ground reference. Defaults reproduce
 * the replay's exact 200 m / 40-cell grid at the origin; the mission
 * preview passes its own extent and centring so the grid always reaches the
 * route being planned.
 */
export function GroundPlane({ size = 200, divisions = 40, position = [0, 0, 0] }: GroundPlaneProps) {
  const [gx, , gz] = position
  return (
    <>
      <gridHelper args={[size, divisions, '#1a2333', '#151d2c']} position={position} />
      {/* Dark base disc so the grid fades into the page background. */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[gx, -0.01, gz]}>
        <circleGeometry args={[size * 0.7, 64]} />
        <meshBasicMaterial color="#0a0e1c" side={THREE.DoubleSide} />
      </mesh>
    </>
  )
}
