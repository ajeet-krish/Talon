import { describe, expect, it } from 'vitest'
import { readFileSync } from 'node:fs'
import { WaypointMission } from './mission'
import type { Waypoint } from './types'
import type { Vec3 } from './math'

interface MissionFixture {
  waypoints: Waypoint[]
  records: {
    position: Vec3
    active_index: number
    complete: boolean
    distance_remaining: number
  }[]
}

function readJson(rel: string): unknown {
  return JSON.parse(
    readFileSync(decodeURIComponent(new URL(rel, import.meta.url).pathname), 'utf8'),
  )
}

const fixture = (readJson('../../fixtures/live_fixtures.json') as { mission: MissionFixture })
  .mission

describe('WaypointMission (TypeScript) matches the Python reference', () => {
  it('advances the active index and latches completion', () => {
    const mission = new WaypointMission(fixture.waypoints)
    for (const record of fixture.records) {
      const complete = mission.update(record.position)
      expect(mission.activeIndex).toBe(record.active_index)
      expect(complete).toBe(record.complete)
      expect(mission.distanceRemaining(record.position)).toBeCloseTo(
        record.distance_remaining,
        9,
      )
    }
  })

  it('rejects an empty waypoint list', () => {
    expect(() => new WaypointMission([])).toThrow()
  })
})
