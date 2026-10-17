import { useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import type * as THREE from 'three'
import type { FlightData } from '@/types/flight'
import { nedToScene, sampleVec3 } from '@/utils/interpolation'
import { useFlightStore } from '@/store/useFlightStore'

/**
 * EKF position uncertainty as a translucent ellipsoid centered on the
 * EKF position estimate. Semi-axes are 3 sigma per NED axis
 * (3 * sqrt(position_var)), mapped to scene axes [N, U, E].
 *
 * This is the estimator's honesty meter: during the GPS outage and the
 * emergency scenario the ellipsoid balloons while the truth trail stays
 * tight, making EKF divergence visible without reading a single number.
 */
export function CovarianceEllipsoid({ data }: { data: FlightData }) {
  const fill = useRef<THREE.Mesh>(null)
  const wire = useRef<THREE.Mesh>(null)
  // Hard cap keeps a blown-up covariance (400 m^2 in emergency) from
  // swallowing the whole scene; it is still ~5x the flight volume.
  const MAX_SCALE = 65

  useFrame(() => {
    const { currentTime, toggles } = useFlightStore.getState()
    const visible = toggles.ellipsoid
    if (!fill.current || !wire.current) return
    fill.current.visible = visible
    wire.current.visible = visible
    if (!visible) return
    const posNed = sampleVec3(currentTime, data.time_s, data.ekf_position)
    const varNed = sampleVec3(currentTime, data.time_s, data.position_var)
    const [x, y, z] = nedToScene(posNed)
    fill.current.position.set(x, y, z)
    wire.current.position.set(x, y, z)
    // NED var order [N, E, D] -> scene [x=N, y=U=-D, z=E].
    const sx = Math.min(3 * Math.sqrt(Math.max(varNed[0], 1e-4)), MAX_SCALE)
    const sy = Math.min(3 * Math.sqrt(Math.max(varNed[2], 1e-4)), MAX_SCALE)
    const sz = Math.min(3 * Math.sqrt(Math.max(varNed[1], 1e-4)), MAX_SCALE)
    fill.current.scale.set(sx, sy, sz)
    wire.current.scale.set(sx, sy, sz)
  })

  return (
    <group>
      <mesh ref={fill}>
        <sphereGeometry args={[1, 28, 18]} />
        <meshBasicMaterial
          color="#26b8d9"
          transparent
          opacity={0.07}
          depthWrite={false}
          side={2 /* THREE.DoubleSide */}
        />
      </mesh>
      <mesh ref={wire}>
        <sphereGeometry args={[1, 16, 10]} />
        <meshBasicMaterial
          color="#26b8d9"
          wireframe
          transparent
          opacity={0.16}
          depthWrite={false}
        />
      </mesh>
    </group>
  )
}
