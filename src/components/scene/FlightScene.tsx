import { Suspense } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import { EffectComposer, Bloom } from '@react-three/postprocessing'
import { useFlightStore } from '@/store/useFlightStore'
import { usePlayback } from '@/hooks/usePlayback'
import { GroundPlane } from '@/components/scene/GroundPlane'
import { DroneMarker, CameraRig } from '@/components/scene/DroneMarker'
import { TrajectoryRibbon } from '@/components/scene/TrajectoryRibbon'
import { CovarianceEllipsoid } from '@/components/scene/CovarianceEllipsoid'
import { WindParticles } from '@/components/scene/WindParticles'
import { WaypointMarkers } from '@/components/scene/WaypointMarkers'
import { ObstacleField } from '@/components/scene/ObstacleField'

/** Advances the store clock from the render loop; renders nothing. */
function PlaybackDriver() {
  usePlayback()
  return null
}

/**
 * Full-bleed replay scene: instrument-grid ground, phase-colored truth
 * trail, quadrotor marker, EKF covariance ellipsoid, wind field, mission
 * waypoints, and spherical keep-out zones. A restrained bloom pass lifts
 * only the bright emissive accents (drone nose, waypoint tags, keep-out
 * shells) so the glow reads as energy, not haze. Camera framing is computed
 * per scenario on load.
 */
export function FlightScene() {
  const data = useFlightStore((s) => s.scenarios.get(s.activeScenario))

  return (
    <Canvas
      dpr={[1, 2]}
      gl={{ antialias: true }}
      camera={{ fov: 42, near: 0.1, far: 800, position: [60, 40, 60] }}
      style={{ position: 'absolute', inset: 0 }}
    >
      <color attach="background" args={['#0b1020']} />
      <fog attach="fog" args={['#0b1020', 90, 340]} />
      <ambientLight intensity={0.7} />
      <directionalLight position={[40, 70, 25]} intensity={1.15} />
      <directionalLight position={[-30, 20, -40]} intensity={0.25} color="#56B4E9" />
      <PlaybackDriver />
      <OrbitControls
        makeDefault
        enableDamping
        dampingFactor={0.08}
        minDistance={5}
        maxDistance={260}
        maxPolarAngle={Math.PI * 0.495}
      />
      {data && (
        <Suspense fallback={null}>
          <CameraRig data={data} />
          <GroundPlane />
          <TrajectoryRibbon data={data} />
          <DroneMarker data={data} />
          <CovarianceEllipsoid data={data} />
          <WindParticles data={data} />
          <WaypointMarkers data={data} />
          <ObstacleField data={data} />
        </Suspense>
      )}
      <EffectComposer>
        <Bloom
          intensity={0.55}
          luminanceThreshold={0.6}
          luminanceSmoothing={0.25}
          mipmapBlur
        />
      </EffectComposer>
    </Canvas>
  )
}
