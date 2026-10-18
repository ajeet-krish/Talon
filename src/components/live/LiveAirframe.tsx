/**
 * The live vehicle: a quadrotor drawn in body axes, tilted by the true attitude.
 *
 * Body x is forward, y is right, z is down, so the model is built nose-along-x
 * and the session's truth quaternion is applied directly. That is the point of
 * drawing it this way: during cruise the airframe visibly noses down, in a wind
 * hold it leans into the disturbance, and the roll and pitch in the rail agree
 * with what the picture does. A nose that only yawed to the ground track would
 * hide the attitude the estimator is doing work to hold.
 *
 * Two altitude cues travel with the vehicle: a soft shadow on the ground plane
 * and a thin stem from the vehicle down to it. Altitude is the quantity that
 * does not read from a perspective camera on its own, so it is stated twice.
 */

import { useMemo, useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import { nedToScene } from '@/utils/interpolation'
import { emptyFrame, type LiveBuffer } from '@/live/stream'
import { SCENE_FROM_NED } from '@/live/view'

const scratch = emptyFrame()

/** Rotor arm angles in the body x-y plane (a plus frame, four arms). */
const ARM_ANGLES = [Math.PI / 4, (3 * Math.PI) / 4, (5 * Math.PI) / 4, (7 * Math.PI) / 4]

/** Scene-from-NED rotation, applied after the body-to-NED attitude. */
const SCENE_QUAT = new THREE.Quaternion(
  SCENE_FROM_NED[0],
  SCENE_FROM_NED[1],
  SCENE_FROM_NED[2],
  SCENE_FROM_NED[3],
)

export function LiveAirframe({ buffer }: { buffer: LiveBuffer }) {
  const body = useRef<THREE.Group>(null)
  const anchor = useRef<THREE.Group>(null)
  const shadow = useRef<THREE.Mesh>(null)
  const stem = useRef<THREE.Mesh>(null)
  const rotors = useRef<(THREE.Group | null)[]>([])
  const q = useMemo(() => new THREE.Quaternion(), [])

  useFrame((_, delta) => {
    const g = body.current
    const a = anchor.current
    if (!g || !a) return
    if (!buffer.poseAt(buffer.clock, scratch)) {
      g.visible = false
      a.visible = false
      return
    }
    g.visible = true
    a.visible = true

    const [x, y, z] = nedToScene(scratch.truthPosition)
    g.position.set(x, y, z)
    a.position.set(x, 0, z)

    const [qw, qx, qy, qz] = scratch.quaternion
    q.set(qx, qy, qz, qw).normalize()
    // body -> NED first, then NED -> scene.
    g.quaternion.copy(SCENE_QUAT).multiply(q)

    // Rotor discs spin with the delivered thrust, so a hover reads as work.
    const spin = 8 + Math.min(scratch.thrust / 9.81, 4) * 22
    for (const rotor of rotors.current) {
      if (rotor) rotor.rotation.z += delta * spin
    }

    const s = shadow.current
    if (s) {
      const height = Math.max(y, 0)
      const spread = 0.7 + height * 0.03
      s.scale.set(spread, spread, 1)
      const mat = s.material as THREE.MeshBasicMaterial
      mat.opacity = Math.max(0, 0.32 * (1 - height / 40))
      s.visible = mat.opacity > 0.01
    }

    const st = stem.current
    if (st) {
      st.visible = y > 0.4
      if (st.visible) {
        st.scale.y = Math.max(y, 0.001)
        st.position.y = y / 2
      }
    }
  })

  return (
    <>
      {/* Ground shadow and altitude stem: position only, never attitude. */}
      <group ref={anchor}>
        <mesh ref={shadow} rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.02, 0]}>
          <circleGeometry args={[1, 32]} />
          <meshBasicMaterial color="#000000" transparent opacity={0.3} depthWrite={false} />
        </mesh>
        <mesh ref={stem}>
          <cylinderGeometry args={[0.012, 0.012, 1, 6]} />
          <meshBasicMaterial color="#26b8d9" transparent opacity={0.3} depthWrite={false} />
        </mesh>
      </group>

      <group ref={body}>
        {/* Fuselage, nose along body +x. */}
        <mesh>
          <boxGeometry args={[0.26, 0.09, 0.22]} />
          <meshStandardMaterial color="#c8d3e0" metalness={0.55} roughness={0.35} />
        </mesh>
        <mesh position={[0.24, 0.01, 0]}>
          <boxGeometry args={[0.14, 0.05, 0.09]} />
          <meshStandardMaterial
            color="#26b8d9"
            emissive="#26b8d9"
            emissiveIntensity={0.7}
            metalness={0.2}
            roughness={0.4}
          />
        </mesh>
        {ARM_ANGLES.map((theta, i) => (
          <group key={i} rotation={[0, 0, theta]}>
            <mesh position={[0.33, 0, 0]}>
              <boxGeometry args={[0.62, 0.05, 0.032]} />
              <meshStandardMaterial color="#8b9bb4" metalness={0.4} roughness={0.5} />
            </mesh>
            <group
              position={[0.64, 0, 0.015]}
              ref={(el) => {
                rotors.current[i] = el
              }}
            >
              <mesh rotation={[Math.PI / 2, 0, 0]}>
                <cylinderGeometry args={[0.17, 0.17, 0.014, 20]} />
                <meshStandardMaterial
                  color="#55657f"
                  metalness={0.3}
                  roughness={0.6}
                  transparent
                  opacity={0.85}
                />
              </mesh>
              <mesh>
                <boxGeometry args={[0.32, 0.022, 0.008]} />
                <meshBasicMaterial color="#9fb0c8" transparent opacity={0.7} />
              </mesh>
            </group>
          </group>
        ))}
      </group>
    </>
  )
}
