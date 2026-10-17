import * as THREE from 'three'

/**
 * 200 m instrument grid on the y = 0 plane, steel lines on the panel
 * blue-black. The drone's soft shadow ellipse is drawn by DroneMarker
 * (it tracks the vehicle); this component is the static ground reference.
 */
export function GroundPlane() {
  return (
    <>
      <gridHelper
        args={[200, 40, '#1a2333', '#151d2c']}
        position={[0, 0, 0]}
      />
      {/* Dark base disc so the grid fades into the page background. */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.01, 0]}>
        <circleGeometry args={[140, 64]} />
        <meshBasicMaterial color="#0a0e1c" side={THREE.DoubleSide} />
      </mesh>
    </>
  )
}
