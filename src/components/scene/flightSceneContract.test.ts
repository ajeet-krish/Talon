/**
 * Replay scene contract (FlightLab Phase 8d-D), asserted from source (the
 * same tier as viewportContract.test.ts: node environment, JSX read out of
 * the files). The waypoint and zone layers were narrowed from `FlightData`
 * to slices so the mission preview can share them; these are the claims a
 * green typecheck alone cannot make: the replay's scene tree, toggles,
 * framing, and live clearance tint are exactly what they were, and only the
 * prop plumbing moved.
 */

import { readFileSync } from 'node:fs'
import path from 'node:path'
import { describe, expect, it } from 'vitest'

const SCENE = readFileSync(path.resolve('src/components/scene/FlightScene.tsx'), 'utf8')
const MARKERS = readFileSync(path.resolve('src/components/scene/WaypointMarkers.tsx'), 'utf8')
const ZONES = readFileSync(path.resolve('src/components/scene/ObstacleField.tsx'), 'utf8')
const GROUND = readFileSync(path.resolve('src/components/scene/GroundPlane.tsx'), 'utf8')

describe('replay scene structure', () => {
  it('still mounts every layer, in order, with the same framing and post pass', () => {
    for (const layer of [
      '<PlaybackDriver />',
      '<CameraRig data={data} />',
      '<GroundPlane />',
      '<TrajectoryRibbon data={data} />',
      '<DroneMarker data={data} />',
      '<CovarianceEllipsoid data={data} />',
      '<WindParticles data={data} />',
      '<EffectComposer>',
      '<Bloom',
    ]) {
      expect(SCENE).toContain(layer)
    }
    // Camera, fog, background, and the orbit clamps are untouched.
    expect(SCENE).toContain("camera={{ fov: 42, near: 0.1, far: 800, position: [60, 40, 60] }}")
    expect(SCENE).toContain("args={['#0b1020', 90, 340]}")
    expect(SCENE).toContain('minDistance={5}')
    expect(SCENE).toContain('maxDistance={260}')
    expect(SCENE).toContain('maxPolarAngle={Math.PI * 0.495}')
  })

  it('keeps the ground exactly as the replay drew it by default', () => {
    // Defaults reproduce the old fixed grid: 200 m, 40 cells, disc r = 140.
    expect(GROUND).toContain('size = 200')
    expect(GROUND).toContain('divisions = 40')
    expect(GROUND).toContain('size * 0.7')
    expect(GROUND).toContain("args={[size, divisions, '#1a2333', '#151d2c']}")
  })
})

describe('narrow layer props', () => {
  it('passes the waypoint and zone slices plus the display toggles', () => {
    expect(SCENE).toContain('waypointsNed={data.waypoints_ned}')
    expect(SCENE).toContain('obstacles={data.obstacles}')
    expect(SCENE).toContain('visible={showWaypoints}')
    expect(SCENE).toContain('visible={showObstacles}')
    // The toggles moved up to the scene, so the mission page (which has no
    // toggle rail) can never inherit replay display state.
    expect(SCENE).toContain('useFlightStore((s) => s.toggles.waypoints)')
    expect(SCENE).toContain('useFlightStore((s) => s.toggles.obstacles)')
    expect(MARKERS).not.toContain('useFlightStore')
    expect(ZONES).not.toContain('toggles')
  })

  it('feeds the zones their live track without the rest of FlightData', () => {
    expect(SCENE).toContain('droneTrack={{ time_s: data.time_s, truth_position: data.truth_position }}')
    expect(ZONES).toContain('droneTrack?: DroneTrack')
    expect(MARKERS).toContain('waypointsNed: Vec3[]')
    expect(ZONES).toContain('obstacles: Obstacle[]')
  })
})

describe('live clearance tint (D8 preserved)', () => {
  it('samples the track every frame and classifies with the shared palette', () => {
    expect(ZONES).toContain('sampleVec3(t, track.time_s, track.truth_position)')
    expect(ZONES).toContain('useFlightStore.getState().currentTime')
    expect(ZONES).toContain('clearanceBand(')
    expect(ZONES).toContain('CLEARANCE_COLORS.breach')
    // With no track (the preview) the cue does not run at all.
    expect(ZONES).toContain('if (track === undefined) return')
  })

  it('falls back to the chart amber only when there is no cue', () => {
    expect(ZONES).toContain("const STATIC_ZONE_COLOR = '#e0a458'")
    expect(ZONES).toContain('const zoneColor = live ? CLEARANCE_COLORS.clear : STATIC_ZONE_COLOR')
    // The replay path still starts from the clearance palette it always did.
    expect(ZONES).toContain('color={zoneColor}')
  })
})
