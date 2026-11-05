/**
 * Mission preview contract (FlightLab Phase 8d-D), asserted from source
 * (the same tier as viewportContract.test.ts: vitest runs environment
 * 'node', so the JSX is read out of the files rather than rendered). Claims
 * a pure test cannot see: the preview subscribes to exactly the four store
 * slices it needs and writes none of them (read-only by construction), it
 * re-frames only on the chart's triggers, it shares the chart's drag
 * transient rather than keeping a private copy, no listener it owns can
 * reach the chart's wheel, and the WebGL-less path is a quiet message
 * rather than a crash.
 */

import { readdirSync, readFileSync, statSync } from 'node:fs'
import path from 'node:path'
import { describe, expect, it } from 'vitest'

const PREVIEW = readFileSync(path.resolve('src/components/scene/MissionPreview.tsx'), 'utf8')
const CHART = readFileSync(path.resolve('src/components/editor/MissionCanvas.tsx'), 'utf8')

describe('mission preview state sharing', () => {
  it('subscribes to the draft, the selection, the transient, and the epoch', () => {
    expect(PREVIEW).toContain('useMissionStore((s) => s.history.present)')
    expect(PREVIEW).toContain('useMissionStore((s) => s.selected)')
    expect(PREVIEW).toContain('useMissionStore((s) => s.dragPreview)')
    expect(PREVIEW).toContain('useMissionStore((s) => s.documentEpoch)')
  })

  it('never writes the mission: no dispatch, no selection, no transient', () => {
    expect(PREVIEW).not.toContain('s.dispatch')
    expect(PREVIEW).not.toContain('.select(')
    expect(PREVIEW).not.toContain('setDragPreview')
    expect(PREVIEW).not.toContain('dispatch(')
    // Pointer input here belongs to OrbitControls: nothing in the panel
    // carries an editor command at all.
    expect(PREVIEW).not.toContain('EditorCommand')
    // Every mention of the store is the import plus the four selector
    // subscriptions: a getState or setState call would be a write path
    // none of the guards above name.
    expect(PREVIEW.match(/useMissionStore/g) ?? []).toHaveLength(6)
    expect(PREVIEW).not.toContain('useMissionStore.getState')
    expect(PREVIEW).not.toContain('useMissionStore.setState')
  })

  it('draws the same transient the chart draws, through the one applier', () => {
    expect(PREVIEW).toContain('withDragPreview(config, dragPreview)')
    expect(CHART).toContain('const drawn = withDragPreview(config, dragPreview)')
  })
})

describe('mission preview camera', () => {
  it('re-frames only on mount, the document epoch, and its own FIT button', () => {
    // The rig's effect: the chart's two triggers plus the local fit signal.
    expect(PREVIEW).toContain('[documentEpoch, fitSignal, controls]')
    // Bounds are read from the trigger's render, never listed as a trigger:
    // a waypoint edit or a drag must not move a camera the user has aimed.
    expect(PREVIEW).not.toContain('[documentEpoch, fitSignal, bounds')
    expect(PREVIEW).toContain('setFitSignal((n) => n + 1)')
    // The zoom clamps travel with the pose, set imperatively so an edit
    // never rescales them either.
    expect(PREVIEW).toContain('controls.minDistance = pose.minDistance')
    expect(PREVIEW).toContain('controls.maxDistance = pose.maxDistance')
  })

  it('orbits and zooms with damping, above the ground, and damps only when motion is allowed', () => {
    expect(PREVIEW).toContain('makeDefault')
    expect(PREVIEW).toContain('enableDamping={!reducedMotion}')
    expect(PREVIEW).toContain('maxPolarAngle={Math.PI * 0.495}')
    expect(PREVIEW).toContain('prefers-reduced-motion')
    // Keyboard orbit listens on the wrapper only, and only when the wrapper
    // itself holds focus, so a focused corner control keeps its own keys.
    expect(PREVIEW).toContain("el.addEventListener('keydown', onKeyDown)")
    expect(PREVIEW).toContain('if (e.target !== el || e.metaKey || e.ctrlKey || e.altKey) return')
  })

  it('poses the camera in exactly one place, so no second effect can re-frame', () => {
    // The trigger list above pins the sanctioned effect; a second effect
    // that re-posed the camera on a config change would satisfy it and
    // still move the camera on every edit. One pose call, one camera set.
    expect(PREVIEW.match(/previewCameraPose\(/g) ?? []).toHaveLength(1)
    expect(PREVIEW.match(/camera\.position\.set\(/g) ?? []).toHaveLength(1)
  })
})

describe('mission preview surface', () => {
  it('is the chart partner, not a replay scene: no rig, no bloom, no lights pass', () => {
    expect(PREVIEW).toContain("import { GroundPlane } from './GroundPlane'")
    expect(PREVIEW).toContain("import { WaypointMarkers } from './WaypointMarkers'")
    expect(PREVIEW).toContain("import { ObstacleField } from './ObstacleField'")
    expect(PREVIEW).not.toContain('CameraRig')
    expect(PREVIEW).not.toContain('postprocessing')
    expect(PREVIEW).not.toContain('EffectComposer')
    expect(PREVIEW).not.toContain('usePlayback')
    expect(PREVIEW).not.toContain('droneTrack') // no telemetry, no live cue
    expect(PREVIEW).not.toContain('useFlightStore') // the authored draft only
  })

  it('renders a quiet fallback when WebGL is missing or the render dies', () => {
    expect(PREVIEW).toContain('supportsWebGL()')
    expect(PREVIEW).toContain('class PreviewBoundary')
    expect(PREVIEW).toContain('3D PREVIEW UNAVAILABLE')
    expect(PREVIEW).toContain('3D PREVIEW STOPPED')
    expect(PREVIEW).toContain('editing control are unaffected')
    // The FIT control only exists with a live camera behind it.
    expect(PREVIEW).toContain('{webgl && (')
  })
})

describe('wheel and pointer isolation', () => {
  it('keeps the chart wheel on the chart svg and the preview wheel on its canvas', () => {
    expect(CHART).toContain("svg.addEventListener('wheel', onWheel, { passive: false })")
    expect(PREVIEW).not.toContain("addEventListener('wheel'")
  })

  /** Every .ts/.tsx file under src, recursively. */
  function sourceFilesUnder(dir: string): string[] {
    const files: string[] = []
    for (const name of readdirSync(dir)) {
      const full = path.join(dir, name)
      if (statSync(full).isDirectory()) files.push(...sourceFilesUnder(full))
      else if (name.endsWith('.ts') || name.endsWith('.tsx')) files.push(full)
    }
    return files
  }

  it('registers no document- or window-level listener anywhere in src', () => {
    // A global listener is how a wheel or a pointer gesture would leak
    // between the chart and the preview; both surfaces attach to their own
    // elements instead.
    const files = sourceFilesUnder(path.resolve('src'))
    expect(files.length).toBeGreaterThan(20) // the walk must actually see the tree
    const offenders = files.filter((file) =>
      /(\bdocument|window)\.addEventListener\(/.test(readFileSync(file, 'utf8')),
    )
    expect(offenders).toEqual([])
  })
})
