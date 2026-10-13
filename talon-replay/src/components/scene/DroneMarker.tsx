import { useEffect, useMemo, useRef } from 'react'
import { useFrame, useThree } from '@react-three/fiber'
import * as THREE from 'three'
import type { FlightData } from '@/types/flight'
import { nedToScene, sampleVec3 } from '@/utils/interpolation'
import { useFlightStore } from '@/store/useFlightStore'

/**
 * Quadrotor marker: four thin arms in an X, rotor discs, a signal-cyan
 * nose so heading reads at a glance. Positioned at the interpolated
 * truth position each frame; yaw follows the ground-track heading
 * (scene x = north, z = east, so yaw = atan2(v_north, v_east)).
 */
export function DroneMarker({ data }: { data: FlightData }) {
  const group = useRef<THREE.Group>(null)
  const shadow = useRef<THREE.Mesh>(null)
  const heading = useRef(0)

  useFrame(() => {
    const g = group.current
    if (!g) return
    const t = useFlightStore.getState().currentTime
    const posNed = sampleVec3(t, data.time_s, data.truth_position)
    const velNed = sampleVec3(t, data.time_s, data.truth_velocity)
    const [x, y, z] = nedToScene(posNed)
    g.position.set(x, y, z)
    const speed = Math.hypot(velNed[0], velNed[1], velNed[2])
    if (speed > 0.05) {
      heading.current = Math.atan2(velNed[0], velNed[2])
    }
    g.rotation.y = heading.current

    // Soft ground shadow ellipse: darker and tighter as the vehicle climbs.
    const s = shadow.current
    if (s) {
      const altitude = Math.max(0, y)
      s.position.set(x, 0.02, z)
      const spread = 0.55 + altitude * 0.02
      s.scale.set(spread, spread, 1)
      const mat = s.material as THREE.MeshBasicMaterial
      mat.opacity = Math.max(0, 0.34 * (1 - altitude / 38))
      s.visible = mat.opacity > 0.01
    }
  })

  const armAngles = useMemo(() => [Math.PI / 4, (3 * Math.PI) / 4, (5 * Math.PI) / 4, (7 * Math.PI) / 4], [])

  return (
    <>
      <mesh ref={shadow} rotation={[-Math.PI / 2, 0, 0]}>
        <circleGeometry args={[1, 32]} />
        <meshBasicMaterial color="#000000" transparent opacity={0.3} depthWrite={false} />
      </mesh>
      <group ref={group} name="drone-marker">
        {/* Fuselage hub */}
        <mesh>
          <boxGeometry args={[0.22, 0.09, 0.26]} />
          <meshStandardMaterial color="#c8d3e0" metalness={0.55} roughness={0.35} />
        </mesh>
        {/* Nose chevron: local +z is the direction of travel */}
        <mesh position={[0, 0.01, 0.24]}>
          <boxGeometry args={[0.09, 0.05, 0.14]} />
          <meshStandardMaterial
            color="#26b8d9"
            emissive="#26b8d9"
            emissiveIntensity={0.55}
            metalness={0.2}
            roughness={0.4}
          />
        </mesh>
        {/* X-frame arms with rotor discs */}
        {armAngles.map((theta, i) => (
          <group key={i} rotation={[0, theta, 0]}>
            <mesh position={[0.33, 0, 0]}>
              <boxGeometry args={[0.62, 0.032, 0.05]} />
              <meshStandardMaterial color="#8b9bb4" metalness={0.4} roughness={0.5} />
            </mesh>
            <mesh position={[0.64, 0.015, 0]} rotation={[0, 0, 0]}>
              <cylinderGeometry args={[0.17, 0.17, 0.014, 20]} />
              <meshStandardMaterial color="#55657f" metalness={0.3} roughness={0.6} transparent opacity={0.85} />
            </mesh>
          </group>
        ))}
      </group>
    </>
  )
}

/** Precompute scene-space truth bounds plus covariance padding, for camera framing. */
export function computeSceneBounds(data: FlightData): {
  center: THREE.Vector3
  radius: number
} {
  const min = new THREE.Vector3(Infinity, Infinity, Infinity)
  const max = new THREE.Vector3(-Infinity, -Infinity, -Infinity)
  for (const ned of data.truth_position) {
    const [x, y, z] = nedToScene(ned)
    min.min(new THREE.Vector3(x, y, z))
    max.max(new THREE.Vector3(x, y, z))
  }
  if (!Number.isFinite(min.x)) {
    min.set(-10, 0, -10)
    max.set(10, 10, 10)
  }
  const center = min.clone().add(max).multiplyScalar(0.5)
  const base = max.clone().sub(min).length() * 0.5
  // Emergency scenarios blow the covariance up to ~60 m radius; pad the
  // framing so the diverging ellipsoid stays readable without manual zoom.
  let maxSigma = 0
  for (const v of data.position_var) {
    maxSigma = Math.max(maxSigma, 3 * Math.sqrt(Math.max(v[0], v[1], v[2])))
  }
  const radius = Math.max(base + Math.min(maxSigma, 24) * 0.6, 12)
  center.y = Math.max(center.y, 2)
  return { center, radius }
}

/** Re-frames the camera and orbit target whenever the scenario changes. */
export function CameraRig({ data }: { data: FlightData }) {
  const camera = useThree((s) => s.camera)
  const controls = useThree((s) => s.controls) as unknown as
    | { target: THREE.Vector3; update: () => void }
    | null

  useEffect(() => {
    const { center, radius } = computeSceneBounds(data)
    camera.position.set(
      center.x + radius * 1.25,
      center.y + radius * 0.85,
      center.z + radius * 1.15,
    )
    camera.lookAt(center)
    camera.updateProjectionMatrix()
    if (controls) {
      controls.target.copy(center)
      controls.update()
    }
  }, [data, camera, controls])

  return null
}
