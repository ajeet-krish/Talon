/**
 * The live ground-station scene.
 *
 * One Canvas holding the ground plane, the planned mission, the revealed
 * trajectory, and the vehicle. Nothing here reads the store clock: the recorder's
 * `clock` field is written by the run view once per frame, and every animated
 * component reads it, so a frame that only advances the clock costs no React
 * work at all.
 *
 * The camera is framed once per scenario from the mission itself. A live
 * trajectory can not frame its own camera: on the first frame it is a single
 * point, so it would zoom to the launch pad and then chase the vehicle as it
 * flew away. The mission is known before the run starts, so the whole flight is
 * in view from the beginning and the operator can still orbit freely.
 */

import { memo, useEffect, useMemo } from 'react'
import { Canvas, useThree } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import { EffectComposer, Bloom } from '@react-three/postprocessing'
import type * as THREE from 'three'
import type { LiveConfig } from '@/live/types'
import type { LiveBuffer } from '@/live/stream'
import { missionBounds, type SceneBounds } from '@/live/view'
import { GroundPlane } from '@/components/scene/GroundPlane'
import { LiveField } from './LiveField'
import { LiveTrail } from './LiveTrail'
import { LiveAirframe } from './LiveAirframe'

/** Places the camera and the orbit target from the mission bounds. */
function LiveCamera({ bounds }: { bounds: SceneBounds }) {
  const camera = useThree((s) => s.camera)
  const controls = useThree((s) => s.controls) as unknown as
    | { target: THREE.Vector3; update: () => void }
    | null

  useEffect(() => {
    const [cx, cy, cz] = bounds.center
    const r = bounds.radius
    camera.position.set(cx + r * 1.35, cy + r * 0.9, cz + r * 1.3)
    camera.lookAt(cx, cy, cz)
    camera.updateProjectionMatrix()
    if (controls) {
      controls.target.set(cx, cy, cz)
      controls.update()
    }
  }, [bounds, camera, controls])

  return null
}

/**
 * Memoised so the rail's 10 Hz text updates do not reconcile the R3F tree. The
 * scene's inputs are the scenario and the recorder, both stable for a run, so a
 * re-render here is always wasted work.
 */
export const LiveScene = memo(function LiveScene({
  cfg,
  buffer,
}: {
  cfg: LiveConfig
  buffer: LiveBuffer
}) {
  const bounds = useMemo(() => missionBounds(cfg), [cfg])

  return (
    <Canvas
      dpr={[1, 2]}
      gl={{ antialias: true }}
      camera={{ fov: 42, near: 0.1, far: 900, position: [60, 45, 60] }}
      style={{ position: 'absolute', inset: 0 }}
    >
      <color attach="background" args={['#0b1020']} />
      <fog attach="fog" args={['#0b1020', 140, 520]} />
      <ambientLight intensity={0.7} />
      <directionalLight position={[40, 70, 25]} intensity={1.15} />
      <directionalLight position={[-30, 20, -40]} intensity={0.25} color="#56B4E9" />
      <LiveCamera bounds={bounds} />
      <OrbitControls
        makeDefault
        enableDamping
        dampingFactor={0.08}
        minDistance={4}
        maxDistance={420}
        maxPolarAngle={Math.PI * 0.495}
      />
      <GroundPlane />
      <LiveField cfg={cfg} buffer={buffer} />
      <LiveTrail buffer={buffer} />
      <LiveAirframe buffer={buffer} />
      <EffectComposer>
        <Bloom intensity={0.5} luminanceThreshold={0.62} luminanceSmoothing={0.25} mipmapBlur />
      </EffectComposer>
    </Canvas>
  )
})
