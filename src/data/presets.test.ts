/**
 * Mission template tests (Phase 8c).
 *
 * The templates are the ids users will actually save and run, so every one
 * must pass the same `validateScenarioConfig` gate the importer and the
 * runner use, and each `build` must return a fresh mission rather than
 * sharing arrays between picks. The teaching preset gets extra pinning
 * because it is the lesson's starting point: 4 to 6 waypoints at 10 m, a
 * visible wind, and exactly one GPS outage.
 */

import { describe, expect, it } from 'vitest'
import { validateScenarioConfig } from '@/data/validate'
import { waypointAltitudeM } from '@/utils/editor'
import { MISSION_TEMPLATES, teachingPreset } from './presets'

describe('MISSION_TEMPLATES', () => {
  it('lists the seven shipped templates in order', () => {
    expect(MISSION_TEMPLATES.map((t) => t.id)).toEqual([
      'empty',
      'hover',
      'waypoint_circuit',
      'obstacle_field',
      'avoidance_demo',
      'wind_hold',
      'teaching',
    ])
  })

  it('builds a valid mission from every template', () => {
    for (const template of MISSION_TEMPLATES) {
      const config = template.build()
      expect(() => validateScenarioConfig(config), template.id).not.toThrow()
      expect(config.id, template.id).toMatch(/^[a-z][a-z0-9_]{0,63}$/)
    }
  })

  it('returns a fresh mission on every build', () => {
    for (const template of MISSION_TEMPLATES) {
      const a = template.build()
      const b = template.build()
      const count = a.waypoints.length
      expect(a, template.id).not.toBe(b)
      expect(a.waypoints, template.id).not.toBe(b.waypoints)
      a.waypoints.push({
        position: [999, 999, -999],
        speed: 1,
        heading: 0,
        radius: 1,
      })
      expect(b.waypoints, template.id).toHaveLength(count)
    }
  })

  it('names every template for the picker', () => {
    for (const template of MISSION_TEMPLATES) {
      expect(template.label.length, template.id).toBeGreaterThan(0)
      expect(template.description.length, template.id).toBeGreaterThan(0)
    }
  })
})

describe('teachingPreset', () => {
  it('is a 4 to 6 waypoint circuit at 10 m', () => {
    const preset = teachingPreset()
    expect(preset.waypoints.length).toBeGreaterThanOrEqual(4)
    expect(preset.waypoints.length).toBeLessThanOrEqual(6)
    for (const wp of preset.waypoints) {
      expect(waypointAltitudeM(wp)).toBe(10)
    }
  })

  it('carries a visible wind and light turbulence', () => {
    const preset = teachingPreset()
    const steady = preset.wind_steady_ned
    const sigma = preset.wind_sigma
    expect(Math.hypot(...steady)).toBeGreaterThanOrEqual(3)
    expect(sigma.some((s) => s > 0)).toBe(true)
  })

  it('schedules exactly one GPS outage at 20 s for 4 s', () => {
    const preset = teachingPreset()
    expect(preset.faults).toHaveLength(1)
    expect(preset.faults[0]).toEqual({ kind: 'gps_outage', at_s: 20, duration_s: 4 })
  })

  it('is a 60 s armed mission under the teaching id', () => {
    const preset = teachingPreset()
    expect(preset.id).toBe('teaching_preset')
    expect(preset.duration_s).toBe(60)
    expect(preset.auto_arm).toBe(true)
    expect(() => validateScenarioConfig(preset)).not.toThrow()
  })

  it('is the same mission the teaching template builds', () => {
    const fromList = MISSION_TEMPLATES.find((t) => t.id === 'teaching')?.build()
    expect(fromList).toEqual(teachingPreset())
  })
})
