/**
 * Run-view config list tests (FlightLab Phase 8c).
 *
 * The authored mission must land ahead of the shipped scenarios without the
 * fetched list being changed or its entries replaced, and the button captions
 * must mark the authored entry as yours while shipped labels pass through
 * untouched. Everything here is pure: no store, no fetch, no DOM.
 */

import { describe, expect, it } from 'vitest'
import type { LiveConfig } from '@/live/types'
import { emptyMission, toMissionConfig } from './editor'
import { configButtonLabel, withAuthoredMission } from './runConfigs'

function config(id: string, label: string): LiveConfig {
  return toMissionConfig({ ...emptyMission(id), label })
}

describe('withAuthoredMission', () => {
  it('returns the fetched list unchanged when nothing was authored', () => {
    const fetched = [config('hover', 'Hover'), config('wind', 'Wind')]
    expect(withAuthoredMission(fetched, null)).toBe(fetched)
  })

  it('returns null while the fetch has not landed and nothing was authored', () => {
    expect(withAuthoredMission(null, null)).toBeNull()
  })

  it('places the authored config first, ahead of every fetched one', () => {
    const authored = config('custom_mission', 'Canyon run')
    const fetched = [config('hover', 'Hover'), config('wind', 'Wind')]
    const merged = withAuthoredMission(fetched, authored)
    expect(merged?.map((c) => c.id)).toEqual(['custom_mission', 'hover', 'wind'])
    expect(merged?.[0]).toBe(authored)
  })

  it('leaves the fetched list and its entries untouched', () => {
    const authored = config('custom_mission', 'Canyon run')
    const hover = config('hover', 'Hover')
    const fetched = [hover]
    const merged = withAuthoredMission(fetched, authored)
    expect(fetched).toEqual([hover])
    expect(fetched).toHaveLength(1)
    expect(merged?.[1]).toBe(hover)
  })

  it('offers the authored mission alone while the fetch has not landed', () => {
    const authored = config('custom_mission', 'Canyon run')
    expect(withAuthoredMission(null, authored)).toEqual([authored])
  })
})

describe('configButtonLabel', () => {
  it('marks the authored entry as yours with its label', () => {
    const authored = config('custom_mission', 'Canyon run')
    const fetched = [config('hover', 'Hover')]
    const merged = withAuthoredMission(fetched, authored) ?? []
    expect(configButtonLabel(merged[0] as LiveConfig, authored)).toBe('MY MISSION: CANYON RUN')
  })

  it('falls back to MY MISSION when the authored label is blank', () => {
    const authored = config('custom_mission', '')
    expect(configButtonLabel(authored, authored)).toBe('MY MISSION')
  })

  it('uppercases shipped labels and leaves them otherwise untouched', () => {
    const authored = config('custom_mission', 'Canyon run')
    const shipped = config('waypoints', 'Waypoints')
    expect(configButtonLabel(shipped, authored)).toBe('WAYPOINTS')
    expect(configButtonLabel(shipped, null)).toBe('WAYPOINTS')
  })

  it('does not mark a shipped config that shares no id with the authored one', () => {
    const authored = config('custom_mission', 'Canyon run')
    const shipped = config('hover', 'MY MISSION decoy')
    expect(configButtonLabel(shipped, authored)).toBe('MY MISSION DECOY')
  })
})
