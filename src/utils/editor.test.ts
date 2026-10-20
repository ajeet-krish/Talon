/**
 * Mission editor model tests (Phase 8c).
 *
 * The editor is the only writer of mission drafts, so the contract pinned
 * here is the one the store, the UI, and the validator all lean on:
 * commands are immutable and total (bad indices are no-ops, never throws),
 * history behaves like a stack with a cap, the estimates match hand-computed
 * flights, and every clone shares nothing with its source. The fault window
 * and the warning strings are asserted exactly because the UI renders them
 * verbatim and the validator enforces the same window.
 */

import { describe, expect, it } from 'vitest'
import type { FaultSpec, Obstacle, ScenarioConfig, Waypoint } from '@/types/generated/talon_core'
import { SCENARIO_DEFAULTS } from '@/types/generated/talon_core'
import {
  applyCommand,
  canRedo,
  canUndo,
  cloneAsTemplate,
  createHistory,
  emptyMission,
  estimateFlightTimeS,
  faultWindowEndS,
  HISTORY_CAP,
  missionWarnings,
  nextMissionId,
  obstacleAltitudeM,
  pointToSegmentDistanceM,
  redo,
  routeDistanceM,
  toMissionConfig,
  undo,
  waypointAltitudeM,
  waypointWarning,
  withObstacleAltitude,
  withWaypointAltitude,
  type EditorCommand,
  type EditorHistory,
  type MissionConfig,
} from './editor'

/** A test mission built from the blank slate. */
function mission(overrides: Partial<MissionConfig> = {}): MissionConfig {
  return { ...emptyMission('test_mission'), ...overrides }
}

function wp(north: number, east: number, down: number, speed = 5, radius = 2): Waypoint {
  return { position: [north, east, down], speed, heading: 0, radius }
}

function zone(id: string, north: number, east: number, down: number, radiusM = 3): Obstacle {
  return { id, center_ned: [north, east, down], radius_m: radiusM }
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

describe('emptyMission', () => {
  it('fills the blank slate from the generated defaults', () => {
    const m = emptyMission()
    expect(m.id).toBe('custom_mission')
    expect(m.label).toBe('Untitled mission')
    expect(m.blurb).toBe('')
    expect(m.duration_s).toBe(60)
    expect(m.takeoff_altitude).toBe(10)
    expect(m.waypoints).toEqual([])
    expect(m.obstacles).toEqual([])
    expect(m.faults).toEqual([])
  })

  it('starts armed with avoidance off and the generated tunings', () => {
    const m = emptyMission()
    expect(m.auto_arm).toBe(true)
    expect(m.avoidance.enabled).toBe(false)
    expect(m.avoidance.clearance_margin_m).toBe(2.0)
    expect(m.avoidance.lookahead_s).toBe(3.0)
    expect(m.avoidance.detour_buffer_m).toBe(0.5)
  })

  it('deep-clones the shared defaults and accepts a custom id', () => {
    const m = emptyMission('my_mission')
    expect(m.id).toBe('my_mission')
    expect(m.start_position_ned).not.toBe(SCENARIO_DEFAULTS.start_position_ned)
    expect(m.wind_sigma).not.toBe(SCENARIO_DEFAULTS.wind_sigma)
    expect(m.wind_steady_ned).not.toBe(SCENARIO_DEFAULTS.wind_steady_ned)
    m.start_position_ned[0] = 99
    m.wind_sigma[1] = 99
    expect(SCENARIO_DEFAULTS.start_position_ned?.[0]).toBe(0)
    expect(SCENARIO_DEFAULTS.wind_sigma?.[1]).toBe(0)
    expect(emptyMission().start_position_ned[0]).toBe(0)
  })
})

describe('toMissionConfig', () => {
  it('completes a sparse config with the generated defaults', () => {
    const sparse: ScenarioConfig = { id: 'sparse_mission', duration_s: 30 }
    const m = toMissionConfig(sparse)
    expect(m.id).toBe('sparse_mission')
    expect(m.duration_s).toBe(30)
    expect(m.takeoff_altitude).toBe(10)
    expect(m.avoidance).toEqual({
      enabled: false,
      clearance_margin_m: 2.0,
      lookahead_s: 3.0,
      detour_buffer_m: 0.5,
    })
    expect(m.auto_arm).toBe(false)
    expect(m.faults).toEqual([])
  })
})

describe('cloneAsTemplate', () => {
  it('clones a shipped config under the new id, armed', () => {
    const shipped: ScenarioConfig = {
      id: 'hover',
      label: 'Hover',
      duration_s: 45,
      auto_arm: false,
      waypoints: [wp(10, 0, -10)],
    }
    const m = cloneAsTemplate(shipped, 'hover_mission')
    expect(m.id).toBe('hover_mission')
    expect(m.label).toBe('Hover')
    expect(m.duration_s).toBe(45)
    expect(m.auto_arm).toBe(true)
    expect(m.waypoints).toHaveLength(1)
    expect(m.avoidance.enabled).toBe(false)
    expect(m.faults).toEqual([])
  })

  it('does not mutate the input and shares nothing with the result', () => {
    const source: ScenarioConfig = {
      id: 'imported',
      waypoints: [wp(5, 5, -10)],
      obstacles: [zone('obs_a', 5, 5, -10)],
      faults: [{ kind: 'gps_outage', at_s: 10, duration_s: 2 }],
    }
    const snapshot = structuredClone(source)
    const m = cloneAsTemplate(source, 'imported_copy')
    expect(source).toEqual(snapshot)
    expect(m.waypoints?.[0]).not.toBe(source.waypoints?.[0])
    expect(m.obstacles?.[0]).not.toBe(source.obstacles?.[0])
    expect(m.faults[0]).not.toBe(source.faults?.[0])
    const first = m.waypoints[0]
    expect(first).toBeDefined()
    if (first) first.position[0] = 999
    expect(source.waypoints?.[0]?.position[0]).toBe(5)
  })
})

// ---------------------------------------------------------------------------
// Altitude conversions (NED z is negative up)
// ---------------------------------------------------------------------------

describe('altitude conversions', () => {
  it('reads waypoint altitude as negative NED z', () => {
    expect(waypointAltitudeM(wp(1, 2, -7.5))).toBe(7.5)
    expect(waypointAltitudeM(wp(0, 0, 0))).toBe(0)
  })

  it('writes waypoint altitude without touching the other fields or the input', () => {
    const original = wp(3, 4, -10)
    const moved = withWaypointAltitude(original, 12)
    expect(moved.position).toEqual([3, 4, -12])
    expect(moved.speed).toBe(5)
    expect(moved.heading).toBe(0)
    expect(moved.radius).toBe(2)
    expect(original.position).toEqual([3, 4, -10])
    expect(moved.position).not.toBe(original.position)
  })

  it('does the same for obstacle altitude', () => {
    const ob = zone('obs_a', 8, 9, -4, 2.5)
    expect(obstacleAltitudeM(ob)).toBe(4)
    const raised = withObstacleAltitude(ob, 15)
    expect(raised.center_ned).toEqual([8, 9, -15])
    expect(raised.id).toBe('obs_a')
    expect(raised.radius_m).toBe(2.5)
    expect(ob.center_ned).toEqual([8, 9, -4])
  })
})

// ---------------------------------------------------------------------------
// Estimates (hand-computed cases)
// ---------------------------------------------------------------------------

describe('routeDistanceM', () => {
  it('is zero with no waypoints', () => {
    expect(routeDistanceM(mission())).toBe(0)
  })

  it('sums start to first waypoint, then leg to leg, horizontal only', () => {
    // Start (0,0) to (3,4): 5 m. Then to (3,14): 10 m. The 10 m climb is
    // vertical and does not count toward the route.
    const cfg = mission({ waypoints: [wp(3, 4, -10), wp(3, 14, -10)] })
    expect(routeDistanceM(cfg)).toBeCloseTo(15, 9)
  })
})

describe('estimateFlightTimeS', () => {
  it('matches the hand-computed flight', () => {
    // Climb 10 m at 2 m/s: 5 s. Holds: one after takeoff plus one per
    // waypoint, 3 x 2 s = 6 s. Legs: 5 m at 2 m/s = 2.5 s, 10 m at 10 m/s
    // = 1 s. Total 5 + 6 + 3.5 = 14.5 s.
    const cfg = mission({
      waypoints: [wp(3, 4, -10, 2), wp(3, 14, -10, 10)],
      cruise_hold_s: 2,
    })
    expect(estimateFlightTimeS(cfg)).toBeCloseTo(14.5, 9)
  })

  it('floors the leg speed at 1 m/s', () => {
    const cfg = mission({ waypoints: [wp(5, 0, -10, 0)], cruise_hold_s: 0 })
    // Climb 5 s + 5 m at the 1 m/s floor = 5 s, no holds.
    expect(estimateFlightTimeS(cfg)).toBeCloseTo(10, 9)
  })

  it('is climb plus a single hold when there is no route', () => {
    const cfg = mission({ takeoff_altitude: 10, cruise_hold_s: 2 })
    expect(estimateFlightTimeS(cfg)).toBeCloseTo(7, 9)
  })
})

// ---------------------------------------------------------------------------
// Fault window (the single source the UI must use)
// ---------------------------------------------------------------------------

describe('faultWindowEndS', () => {
  it('puts the last step on the grid for a whole run', () => {
    // 60 s at dt 0.005 is 12000 steps; the last step starts at 59.995 s.
    expect(faultWindowEndS(60, 0.005)).toBeCloseTo(59.995, 9)
  })

  it('handles durations that fall between grid points', () => {
    // 0.02 s at dt 0.01 rounds to 2 steps; the last step starts at 0.01 s.
    expect(faultWindowEndS(0.02, 0.01)).toBeCloseTo(0.01, 9)
    // 0.008 s rounds to 2 steps: the window opens at one step.
    expect(faultWindowEndS(0.008, 0.005)).toBeCloseTo(0.005, 9)
  })

  it('returns null when the run has fewer than two steps', () => {
    expect(faultWindowEndS(0.001, 0.005)).toBeNull()
    expect(faultWindowEndS(0.004, 0.005)).toBeNull()
    expect(faultWindowEndS(0.006, 0.005)).toBeNull()
    expect(faultWindowEndS(0, 0.005)).toBeNull()
  })

  it('agrees with the validator on a shipped window', () => {
    // The teaching preset schedules its outage at 20 s, well inside 59.995.
    expect(faultWindowEndS(60, 0.005) ?? 0).toBeGreaterThan(20)
  })
})

// ---------------------------------------------------------------------------
// Warnings (rendered verbatim by the UI)
// ---------------------------------------------------------------------------

describe('waypointWarning', () => {
  it('names the zone a waypoint sits inside', () => {
    const cfg = mission({
      waypoints: [wp(20, 0, -10)],
      obstacles: [zone('obs_north', 20, 3, -10, 4)],
    })
    expect(waypointWarning(cfg, 0)).toBe('Waypoint 1 is inside keep-out zone obs_north')
  })

  it('flags a duplicate of the waypoint before it', () => {
    const cfg = mission({ waypoints: [wp(10, 10, -10), wp(10, 10.05, -10)] })
    expect(waypointWarning(cfg, 1)).toBe('Waypoint 2 repeats waypoint 1')
  })

  it('flags an acceptance radius that covers the previous waypoint', () => {
    const near = mission({ waypoints: [wp(0, 0, -10), wp(3, 0, -10, 5, 2.9)] })
    expect(waypointWarning(near, 1)).toBeNull()
    const covering = mission({ waypoints: [wp(0, 0, -10), wp(3, 0, -10, 5, 3)] })
    expect(waypointWarning(covering, 1)).toBe("Waypoint 2's radius covers waypoint 1")
  })

  it('stays quiet on a clean route and off the end of the list', () => {
    const cfg = mission({ waypoints: [wp(0, 0, -10), wp(10, 0, -10), wp(10, 10, -10)] })
    expect(waypointWarning(cfg, 0)).toBeNull()
    expect(waypointWarning(cfg, 1)).toBeNull()
    expect(waypointWarning(cfg, 2)).toBeNull()
    expect(waypointWarning(cfg, 7)).toBeNull()
  })

  it('checks the zone before the duplicates so the blocking fact leads', () => {
    const cfg = mission({
      waypoints: [wp(20, 3, -10), wp(20, 3, -10)],
      obstacles: [zone('obs_a', 20, 3, -10, 4)],
    })
    expect(waypointWarning(cfg, 1)).toBe('Waypoint 2 is inside keep-out zone obs_a')
  })
})

describe('pointToSegmentDistanceM', () => {
  it('measures the perpendicular to a leg the point is level with', () => {
    expect(pointToSegmentDistanceM([7.5, 4, -10], [0, 0, -10], [15, 0, -10])).toBe(4)
    expect(pointToSegmentDistanceM([7.5, 0, 0], [0, 0, 0], [15, 0, 0])).toBe(0)
  })

  it('clamps to the nearer end once the point is past the leg', () => {
    expect(pointToSegmentDistanceM([25, 3, -10], [0, 0, -10], [15, 0, -10])).toBeCloseTo(
      Math.hypot(10, 3),
      9,
    )
    expect(pointToSegmentDistanceM([-6, -8, -10], [0, 0, -10], [15, 0, -10])).toBe(10)
  })

  it('ignores altitude and treats a zero-length leg as a point', () => {
    expect(pointToSegmentDistanceM([3, 4, 999], [1, 1, 1], [1, 1, 1])).toBeCloseTo(
      Math.hypot(2, 3),
      9,
    )
  })
})

describe('missionWarnings', () => {
  /** The tester's geometry: a zone centred on a short leg, avoidance on. */
  function onShortLeg(overrides: Partial<MissionConfig> = {}): MissionConfig {
    return mission({
      waypoints: [wp(15, 0, -10, 5, 2)],
      obstacles: [zone('zone_1', 7.5, 0, -10, 3)],
      avoidance: {
        enabled: true,
        clearance_margin_m: 2,
        lookahead_s: 3,
        detour_buffer_m: 0.5,
      },
      ...overrides,
    })
  }

  const byPassWarning =
    'Keep-out zone zone_1 is close to the leg to waypoint 1. ' +
    'The bypass may carry the vehicle past waypoint 1: use a larger acceptance radius ' +
    'or move the zone.'

  it('warns on a zone centred on a short leg', () => {
    expect(missionWarnings(onShortLeg())).toEqual([byPassWarning])
  })

  it('stays quiet when the zone is far from every leg', () => {
    const far = onShortLeg({ obstacles: [zone('zone_1', 7.5, 40, -10, 3)] })
    expect(missionWarnings(far)).toEqual([])
  })

  it('stays quiet when avoidance is off, however close the zone is', () => {
    const off = onShortLeg({
      avoidance: {
        enabled: false,
        clearance_margin_m: 2,
        lookahead_s: 3,
        detour_buffer_m: 0.5,
      },
    })
    expect(missionWarnings(off)).toEqual([])
  })

  it('warns even on a leg whose acceptance radius is generous', () => {
    // The warning describes the geometry, not a guarantee: a wide radius may
    // still swallow the overshoot, and the author should still decide.
    const generous = onShortLeg({ waypoints: [wp(15, 0, -10, 5, 12)] })
    expect(missionWarnings(generous)).toEqual([byPassWarning])
  })

  it('warns once for a zone listed twice on the same leg', () => {
    const doubled = onShortLeg({
      obstacles: [zone('zone_1', 7.5, 0, -10, 3), zone('zone_1', 7.5, 0, -10, 3)],
    })
    expect(missionWarnings(doubled)).toEqual([byPassWarning])
  })

  it('carries the per-waypoint warnings ahead of the zone ones', () => {
    const cfg = onShortLeg({ waypoints: [wp(15, 0, -10, 5, 2), wp(15.05, 0, -10, 5, 2)] })
    expect(missionWarnings(cfg)).toEqual(['Waypoint 2 repeats waypoint 1', byPassWarning])
  })

  it('says nothing when there is no route to warn about', () => {
    const noRoute = onShortLeg({ waypoints: [] })
    expect(missionWarnings(noRoute)).toEqual([])
  })
})

// ---------------------------------------------------------------------------
// Id generation
// ---------------------------------------------------------------------------

describe('nextMissionId', () => {
  it('offers the base id first', () => {
    expect(nextMissionId([])).toBe('custom_mission')
  })

  it('steps through collisions', () => {
    expect(nextMissionId(['custom_mission'])).toBe('custom_mission_2')
    expect(nextMissionId(['custom_mission', 'custom_mission_2'])).toBe('custom_mission_3')
    expect(
      nextMissionId(['custom_mission', 'custom_mission_2', 'custom_mission_3']),
    ).toBe('custom_mission_4')
  })

  it('ignores reserved shipped ids and unrelated used ids', () => {
    expect(nextMissionId(['hover', 'wind', 'live', 'some_other_draft'])).toBe('custom_mission')
  })
})

// ---------------------------------------------------------------------------
// Commands: waypoints
// ---------------------------------------------------------------------------

function dispatch(history: EditorHistory, ...commands: Parameters<typeof applyCommand>[1][]): EditorHistory {
  return commands.reduce((h, cmd) => applyCommand(h, cmd), history)
}

describe('applyCommand: waypoints', () => {
  it('adds a waypoint at the end with the editor defaults', () => {
    const h = dispatch(createHistory(mission()), { type: 'waypoint/add', north: 12, east: -4 })
    const added = h.present.waypoints[0]
    expect(h.present.waypoints).toHaveLength(1)
    expect(added?.position).toEqual([12, -4, -10]) // altitude defaults to takeoff
    expect(added?.speed).toBe(5)
    expect(added?.heading).toBe(0)
    expect(added?.radius).toBe(2)
  })

  it('inserts at a given index and honors altitude and speed', () => {
    const start = mission({ waypoints: [wp(0, 20, -10), wp(0, 40, -10)] })
    const h = dispatch(createHistory(start), {
      type: 'waypoint/add',
      north: 0,
      east: 30,
      index: 1,
      altitude: 15,
      speed: 7,
    })
    expect(h.present.waypoints.map((w) => w.position[1])).toEqual([20, 30, 40])
    expect(h.present.waypoints[1]?.position).toEqual([0, 30, -15])
    expect(h.present.waypoints[1]?.speed).toBe(7)
  })

  it('rejects an insert index past the end (no-op, same history)', () => {
    const h = createHistory(mission({ waypoints: [wp(0, 0, -10)] }))
    const after = applyCommand(h, { type: 'waypoint/add', north: 1, east: 1, index: 5 })
    expect(after).toBe(h)
  })

  it('moves a waypoint horizontally and keeps its altitude', () => {
    const start = mission({ waypoints: [wp(0, 0, -12), wp(10, 0, -10)] })
    const h = dispatch(createHistory(start), { type: 'waypoint/move', index: 0, north: 5, east: 6 })
    expect(h.present.waypoints[0]?.position).toEqual([5, 6, -12])
    expect(h.present.waypoints[1]?.position).toEqual([10, 0, -10])
  })

  it('replaces a whole waypoint with set', () => {
    const start = mission({ waypoints: [wp(0, 0, -10)] })
    const h = dispatch(createHistory(start), {
      type: 'waypoint/set',
      index: 0,
      waypoint: wp(9, 9, -3, 8, 1),
    })
    expect(h.present.waypoints[0]).toEqual({ position: [9, 9, -3], speed: 8, heading: 0, radius: 1 })
  })

  it('removes one waypoint and no-ops off the end', () => {
    const start = mission({ waypoints: [wp(0, 0, -10), wp(10, 0, -10), wp(20, 0, -10)] })
    const h = dispatch(createHistory(start), { type: 'waypoint/remove', index: 1 })
    expect(h.present.waypoints.map((w) => w.position[0])).toEqual([0, 20])
    const untouched = applyCommand(h, { type: 'waypoint/remove', index: 9 })
    expect(untouched).toBe(h)
  })

  it('swaps with the neighbor when reordering, and no-ops at the ends', () => {
    const start = mission({ waypoints: [wp(0, 0, -10), wp(10, 0, -10), wp(20, 0, -10)] })
    const h = dispatch(createHistory(start), { type: 'waypoint/reorder', index: 0, delta: 1 })
    expect(h.present.waypoints.map((w) => w.position[0])).toEqual([10, 0, 20])
    const up = applyCommand(h, { type: 'waypoint/reorder', index: 0, delta: -1 })
    expect(up).toBe(h)
    const down = applyCommand(h, { type: 'waypoint/reorder', index: 2, delta: 1 })
    expect(down).toBe(h)
  })
})

// ---------------------------------------------------------------------------
// Commands: obstacles
// ---------------------------------------------------------------------------

describe('applyCommand: obstacles', () => {
  it('adds a zone with a generated unique id', () => {
    const h = dispatch(createHistory(mission()), {
      type: 'obstacle/add',
      north: 15,
      east: 5,
    })
    expect(h.present.obstacles[0]).toEqual({
      id: 'zone_1',
      center_ned: [15, 5, -10],
      radius_m: 3,
    })
    const h2 = dispatch(h, { type: 'obstacle/add', north: 0, east: 0, radius_m: 4.5 })
    expect(h2.present.obstacles[1]?.id).toBe('zone_2')
  })

  it('skips an id that is already taken', () => {
    const start = mission({ obstacles: [zone('zone_1', 0, 0, -10)] })
    const h = dispatch(createHistory(start), { type: 'obstacle/add', north: 1, east: 1 })
    expect(h.present.obstacles[1]?.id).toBe('zone_2')
  })

  it('moves a zone horizontally and keeps its altitude', () => {
    const start = mission({ obstacles: [zone('obs_a', 0, 0, -6, 3)] })
    const h = dispatch(createHistory(start), { type: 'obstacle/move', index: 0, north: 4, east: -2 })
    expect(h.present.obstacles[0]?.center_ned).toEqual([4, -2, -6])
  })

  it('resizes a zone and replaces or removes one whole', () => {
    const start = mission({ obstacles: [zone('obs_a', 0, 0, -10), zone('obs_b', 5, 5, -10)] })
    const resized = dispatch(createHistory(start), { type: 'obstacle/resize', index: 0, radius_m: 7 })
    expect(resized.present.obstacles[0]?.radius_m).toBe(7)
    const replaced = dispatch(resized, {
      type: 'obstacle/set',
      index: 1,
      obstacle: zone('obs_b', 9, 9, -2, 1),
    })
    expect(replaced.present.obstacles[1]).toEqual({ id: 'obs_b', center_ned: [9, 9, -2], radius_m: 1 })
    const removed = dispatch(replaced, { type: 'obstacle/remove', index: 0 })
    expect(removed.present.obstacles).toHaveLength(1)
    expect(removed.present.obstacles[0]?.id).toBe('obs_b')
    const untouched = applyCommand(removed, { type: 'obstacle/resize', index: 4, radius_m: 2 })
    expect(untouched).toBe(removed)
  })
})

// ---------------------------------------------------------------------------
// Commands: wind, avoidance, params, faults, reset
// ---------------------------------------------------------------------------

describe('applyCommand: wind and avoidance', () => {
  it('sets steady wind and turbulence sigma, copying the triples', () => {
    const ned: [number, number, number] = [3, 1, 0]
    const h = dispatch(
      createHistory(mission()),
      { type: 'wind/steady', ned },
      { type: 'wind/sigma', ned: [0.5, 0.5, 0.25] },
    )
    expect(h.present.wind_steady_ned).toEqual([3, 1, 0])
    expect(h.present.wind_sigma).toEqual([0.5, 0.5, 0.25])
    ned[0] = 99
    expect(h.present.wind_steady_ned[0]).toBe(3)
  })

  it('flips the avoidance switch and tunes only the given knobs', () => {
    const h = dispatch(
      createHistory(mission()),
      { type: 'avoidance/enabled', enabled: true },
      { type: 'avoidance/tuning', clearance_margin_m: 4, lookahead_s: 2.5 },
    )
    expect(h.present.avoidance).toEqual({
      enabled: true,
      clearance_margin_m: 4,
      lookahead_s: 2.5,
      detour_buffer_m: 0.5,
    })
    const noop = applyCommand(h, { type: 'avoidance/tuning' })
    expect(noop).toBe(h)
  })
})

describe('applyCommand: params', () => {
  it('sets a string param', () => {
    const h = dispatch(
      createHistory(mission()),
      { type: 'param/set', key: 'label', value: 'Delivery loop' },
      { type: 'param/set', key: 'blurb', value: 'Four legs at 15 m.' },
    )
    expect(h.present.label).toBe('Delivery loop')
    expect(h.present.blurb).toBe('Four legs at 15 m.')
  })

  it('sets numeric params including seed, and the auto_arm flag', () => {
    const h = dispatch(
      createHistory(mission()),
      { type: 'param/set', key: 'duration_s', value: 120 },
      { type: 'param/set', key: 'takeoff_altitude', value: 25 },
      { type: 'param/set', key: 'cruise_hold_s', value: 5 },
      { type: 'param/set', key: 'seed', value: 7 },
      { type: 'param/set', key: 'auto_arm', value: false },
    )
    expect(h.present.duration_s).toBe(120)
    expect(h.present.takeoff_altitude).toBe(25)
    expect(h.present.cruise_hold_s).toBe(5)
    expect(h.present.seed).toBe(7)
    expect(h.present.auto_arm).toBe(false)
  })
})

describe('applyCommand: faults and reset', () => {
  it('replaces the fault list and copies its entries', () => {
    const faults: FaultSpec[] = [
      { kind: 'gps_outage', at_s: 20, duration_s: 4 },
      { kind: 'emergency', at_s: 45 },
    ]
    const h = dispatch(createHistory(mission()), { type: 'faults/set', faults })
    expect(h.present.faults).toEqual(faults)
    expect(h.present.faults[0]).not.toBe(faults[0])
    const cleared = dispatch(h, { type: 'faults/set', faults: [] })
    expect(cleared.present.faults).toEqual([])
  })

  it('resets to a given config, copying it', () => {
    const replacement = mission({ id: 'replaced_mission', waypoints: [wp(1, 2, -10)] })
    const h = dispatch(createHistory(mission()), { type: 'reset', config: replacement })
    expect(h.present.id).toBe('replaced_mission')
    expect(h.present).not.toBe(replacement)
    expect(h.present.waypoints[0]).not.toBe(replacement.waypoints[0])
  })

  it('is a no-op when the reset config deep-equals the present', () => {
    const start = mission({
      label: 'Edited',
      waypoints: [wp(1, 2, -10)],
      obstacles: [zone('obs_a', 5, 5, -10)],
      faults: [{ kind: 'rtl', at_s: 5 }],
    })
    const edited = dispatch(createHistory(start), { type: 'param/set', key: 'blurb', value: 'seeded' })
    const h = undo(edited) // present: start, future: [edited], so the redo stack is live
    expect(h.future).toHaveLength(1)

    const copy = toMissionConfig(h.present) // same values, different objects
    const after = applyCommand(h, { type: 'reset', config: copy })
    expect(after).toBe(h) // the same history: no undo entry manufactured
    expect(after.present).toBe(h.present) // and the present was not even cloned
    expect(after.future).toHaveLength(1) // the redo stack survives

    // A config that differs on any field still resets and stacks an entry.
    const changed = applyCommand(h, { type: 'reset', config: { ...copy, label: 'Other' } })
    expect(changed).not.toBe(h)
    expect(changed.past).toHaveLength(h.past.length + 1)
    expect(changed.present.label).toBe('Other')
  })
})

// ---------------------------------------------------------------------------
// History: immutability, cap, undo/redo
// ---------------------------------------------------------------------------

describe('applyCommand: history discipline', () => {
  it('never mutates the incoming history or its config', () => {
    const start = mission({ waypoints: [wp(0, 0, -10)] })
    const snapshot = structuredClone(start)
    const h = createHistory(start)
    const after = applyCommand(h, { type: 'waypoint/add', north: 5, east: 5 })
    expect(start).toEqual(snapshot)
    expect(h.present).toEqual(snapshot)
    expect(h.past).toEqual([])
    expect(after.past).toEqual([snapshot])
  })

  it('clears the redo stack on a new command', () => {
    let h = dispatch(createHistory(mission()), { type: 'param/set', key: 'label', value: 'a' })
    h = undo(h)
    expect(canRedo(h)).toBe(true)
    h = applyCommand(h, { type: 'param/set', key: 'label', value: 'b' })
    expect(h.future).toEqual([])
    expect(h.present.label).toBe('b')
  })

  it('caps the undo stack and drops the oldest entries', () => {
    let h = createHistory(mission())
    for (let i = 0; i < HISTORY_CAP + 5; i += 1) {
      h = applyCommand(h, { type: 'param/set', key: 'cruise_hold_s', value: i })
    }
    expect(h.past).toHaveLength(HISTORY_CAP)
    expect(h.present.cruise_hold_s).toBe(HISTORY_CAP + 4)
    // The five oldest entries (the starting config plus edits 0 through 3)
    // fall off the front; the oldest kept edit set cruise_hold_s to 4.
    expect(h.past[0]?.cruise_hold_s).toBe(4)
  })
})

describe('undo and redo', () => {
  it('walks back and forward through the edits', () => {
    let h = dispatch(
      createHistory(mission()),
      { type: 'waypoint/add', north: 10, east: 0 },
      { type: 'waypoint/add', north: 10, east: 10 },
    )
    expect(h.present.waypoints).toHaveLength(2)
    expect(canUndo(h)).toBe(true)
    expect(canRedo(h)).toBe(false)

    h = undo(h)
    expect(h.present.waypoints).toHaveLength(1)
    h = undo(h)
    expect(h.present.waypoints).toHaveLength(0)
    expect(canUndo(h)).toBe(false)
    expect(undo(h)).toBe(h) // no-op at the start of the stack

    h = redo(h)
    expect(h.present.waypoints).toHaveLength(1)
    h = redo(h)
    expect(h.present.waypoints).toHaveLength(2)
    expect(canRedo(h)).toBe(false)
    expect(redo(h)).toBe(h) // no-op past the end of the stack
  })

  it('restores the very object on redo and keeps stacks distinct', () => {
    let h = dispatch(createHistory(mission()), { type: 'param/set', key: 'duration_s', value: 30 })
    h = undo(h)
    const edited = h.future[0]
    expect(edited?.duration_s).toBe(30)
    h = redo(h)
    expect(h.present).toBe(edited) // redo restores the very object, not a copy
    expect(h.present.duration_s).toBe(30)
    expect(h.past).toHaveLength(1) // the pre-edit state is stacked again
    expect(h.future).toHaveLength(0)
  })
})

// ---------------------------------------------------------------------------
// Commands: a value-equal command is a no-op (it must not manufacture history)
// ---------------------------------------------------------------------------

describe('applyCommand: value-equal commands are no-ops', () => {
  /** A mission with something to compare against in every command group. */
  function base(): MissionConfig {
    return mission({
      label: 'Edited',
      duration_s: 60,
      waypoints: [wp(0, 0, -10, 5, 2), wp(10, 0, -10, 5, 2)],
      obstacles: [zone('obs_a', 20, 5, -10, 3)],
      wind_steady_ned: [2, 1, 0],
      wind_sigma: [0.5, 0.5, 0.25],
      faults: [{ kind: 'gps_outage', at_s: 10, duration_s: 2 }],
    })
  }

  /**
   * A history with one entry on each stack, so the assertions can prove a
   * no-op neither pushes `past` nor clears `future`: two edits, then undo.
   */
  function seeded(): EditorHistory {
    const edited = dispatch(
      createHistory(base()),
      { type: 'param/set', key: 'blurb', value: 'seeded' },
      { type: 'waypoint/add', north: 30, east: 0 },
    )
    return undo(edited) // present: base + blurb, past: 2, future: 1
  }

  /** One command group: the value-equal command and a real change of it. */
  const cases: { name: string; noop: EditorCommand; change: EditorCommand }[] = [
    {
      name: 'param/set',
      noop: { type: 'param/set', key: 'duration_s', value: 60 },
      change: { type: 'param/set', key: 'duration_s', value: 90 },
    },
    {
      name: 'waypoint/move',
      noop: { type: 'waypoint/move', index: 0, north: 0, east: 0 },
      change: { type: 'waypoint/move', index: 0, north: 5, east: 0 },
    },
    {
      name: 'waypoint/set',
      noop: { type: 'waypoint/set', index: 0, waypoint: wp(0, 0, -10, 5, 2) },
      change: { type: 'waypoint/set', index: 0, waypoint: wp(0, 0, -10, 6, 2) },
    },
    {
      name: 'obstacle/move',
      noop: { type: 'obstacle/move', index: 0, north: 20, east: 5 },
      change: { type: 'obstacle/move', index: 0, north: 25, east: 5 },
    },
    {
      name: 'obstacle/resize',
      noop: { type: 'obstacle/resize', index: 0, radius_m: 3 },
      change: { type: 'obstacle/resize', index: 0, radius_m: 4 },
    },
    {
      name: 'obstacle/set',
      noop: { type: 'obstacle/set', index: 0, obstacle: zone('obs_a', 20, 5, -10, 3) },
      change: { type: 'obstacle/set', index: 0, obstacle: zone('obs_a', 20, 5, -10, 4) },
    },
    {
      name: 'wind/steady',
      noop: { type: 'wind/steady', ned: [2, 1, 0] },
      change: { type: 'wind/steady', ned: [3, 1, 0] },
    },
    {
      name: 'wind/sigma',
      noop: { type: 'wind/sigma', ned: [0.5, 0.5, 0.25] },
      change: { type: 'wind/sigma', ned: [0.6, 0.5, 0.25] },
    },
    {
      name: 'avoidance/enabled',
      noop: { type: 'avoidance/enabled', enabled: false },
      change: { type: 'avoidance/enabled', enabled: true },
    },
    {
      // Both provided fields already hold their values: still a no-op.
      name: 'avoidance/tuning',
      noop: { type: 'avoidance/tuning', clearance_margin_m: 2, lookahead_s: 3 },
      change: { type: 'avoidance/tuning', clearance_margin_m: 2, lookahead_s: 4 },
    },
    {
      name: 'faults/set',
      noop: { type: 'faults/set', faults: [{ kind: 'gps_outage', at_s: 10, duration_s: 2 }] },
      change: { type: 'faults/set', faults: [{ kind: 'gps_outage', at_s: 10, duration_s: 3 }] },
    },
  ]

  it.each(cases)(
    'returns the same $name history when the value already equals the present',
    ({ noop }) => {
      const h = seeded()
      const pastLength = h.past.length
      const future = h.future
      expect(future).toHaveLength(1) // the fixture really has a redo stack

      const after = applyCommand(h, noop)
      expect(after).toBe(h) // the same EditorHistory reference
      expect(after.past).toHaveLength(pastLength)
      expect(after.future).toBe(future) // the redo stack is untouched
      expect(after.present).toBe(h.present)
    },
  )

  it.each(cases)('records exactly one entry for a real $name change', ({ change }) => {
    const h = seeded()
    const after = applyCommand(h, change)
    expect(after).not.toBe(h)
    expect(after.past).toHaveLength(h.past.length + 1)
    expect(after.past[after.past.length - 1]).toBe(h.present)
    expect(after.future).toEqual([]) // a real edit consumes the redo stack
    expect(after.present).not.toBe(h.present)
  })

  it('keeps a redo stack alive across a value-equal command', () => {
    let h = dispatch(createHistory(mission()), { type: 'param/set', key: 'label', value: 'a' })
    h = undo(h)
    expect(canRedo(h)).toBe(true)
    const after = applyCommand(h, { type: 'param/set', key: 'label', value: 'Untitled mission' })
    expect(after).toBe(h) // equal to the present label: no new entry, redo intact
    expect(canRedo(after)).toBe(true)
    const changed = applyCommand(h, { type: 'param/set', key: 'label', value: 'b' })
    expect(changed.future).toEqual([])
    expect(changed.present.label).toBe('b')
  })
})
