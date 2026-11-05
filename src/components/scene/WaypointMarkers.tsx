import { useEffect, useMemo } from 'react'
import * as THREE from 'three'
import type { Vec3 } from '@/types/flight'
import { nedToScene } from '@/utils/interpolation'

/** Altitude text for a waypoint, matching the chart's raw number label. */
function altitudeText(ned: Vec3): string {
  return `${-ned[2]}`
}

/**
 * Small canvas-rendered label sprite so numbers stay offline-safe. One line
 * (128x64) is the replay's `WP1` tag; two lines (256x96) add the altitude
 * under it, the same label geometry the keep-out zones use.
 */
function makeLabel(lines: readonly string[]): THREE.CanvasTexture {
  const twoLine = lines.length > 1
  const canvas = document.createElement('canvas')
  canvas.width = twoLine ? 256 : 128
  canvas.height = twoLine ? 96 : 64
  const ctx = canvas.getContext('2d')
  if (ctx) {
    ctx.textAlign = 'center'
    ctx.textBaseline = 'middle'
    ctx.font = '600 34px "IBM Plex Mono", monospace'
    ctx.fillStyle = '#dbe4f0'
    const x = twoLine ? 128 : 64
    ctx.fillText(lines[0] ?? '', x, 32)
    if (twoLine) {
      ctx.font = '500 26px "IBM Plex Mono", monospace'
      ctx.fillStyle = '#9aa9bf'
      ctx.fillText(lines[1] ?? '', x, 70)
    }
  }
  const tex = new THREE.CanvasTexture(canvas)
  tex.colorSpace = THREE.SRGBColorSpace
  return tex
}

export interface WaypointMarkersProps {
  /** The mission's waypoints in NED, drawn as numbered spheres on the route. */
  waypointsNed: Vec3[]
  /**
   * Index of the waypoint the editor points at. Its node fills signal cyan
   * and its ground ring brightens and widens, the way the 2D chart rings the
   * selected node. The replay passes nothing.
   */
  highlight?: number
  /**
   * Also label each waypoint's altitude under its number (the mission
   * preview does; the replay keeps its one-line tag).
   */
  showAltitude?: boolean
  /**
   * Layer visibility. The replay passes its display toggle; the preview
   * leaves it at the default, so the mission page never inherits the
   * replay's toggle state.
   */
  visible?: boolean
}

/**
 * Mission waypoints as numbered spheres, positioned from NED through
 * `nedToScene`. Input is a narrow slice rather than the whole `FlightData`,
 * so the same layer serves the replay (passing `data.waypoints_ned`) and
 * the mission preview (passing the draft's waypoints, drag included).
 * Renders nothing when the mission list is empty.
 */
export function WaypointMarkers({
  waypointsNed,
  highlight,
  showAltitude = false,
  visible = true,
}: WaypointMarkersProps) {
  const waypoints = useMemo(
    () => waypointsNed.map((ned, i) => ({ key: i, scene: nedToScene(ned) })),
    [waypointsNed],
  )

  /**
   * Texture cache keyed on what the label text depends on: the indices and,
   * when shown, the altitudes. A position edit or a live drag leaves the
   * key untouched, so dragging reuses the canvases instead of redrawing
   * them at pointer-move rate.
   */
  const labelKey = showAltitude
    ? waypointsNed.map((ned, i) => `${i + 1}:${altitudeText(ned)}`).join(' ')
    : `count:${waypointsNed.length}`
  const textures = useMemo(
    () =>
      waypointsNed.map((ned, i) =>
        makeLabel(showAltitude ? [`WP${i + 1}`, `${altitudeText(ned)} m`] : [`WP${i + 1}`]),
      ),
    [labelKey], // labelKey covers the count, the altitude flag, and every altitude
  )

  useEffect(() => {
    return () => {
      for (const tex of textures) tex.dispose()
    }
  }, [textures])

  if (waypoints.length === 0) return null

  return (
    <group visible={visible}>
      {waypoints.map((w, i) => {
        const hot = highlight === i
        // Sprite scale tracks the label canvas aspect: 128/64 for one line,
        // 256/96 for two, so the text reads at the same size either way.
        return (
          <group key={w.key} position={[w.scene[0], w.scene[1], w.scene[2]]}>
            <mesh>
              <sphereGeometry args={[0.35, 20, 14]} />
              <meshBasicMaterial color={hot ? '#26b8d9' : '#dbe4f0'} transparent opacity={0.9} />
            </mesh>
            <mesh
              position={[0, -w.scene[1] + 0.03, 0]}
              rotation={[-Math.PI / 2, 0, 0]}
              scale={hot ? 1.6 : 1}
            >
              <ringGeometry args={[0.45, 0.58, 28]} />
              <meshBasicMaterial
                color="#26b8d9"
                transparent
                opacity={hot ? 0.95 : 0.6}
                side={2}
              />
            </mesh>
            <sprite position={[0, 1.2, 0]} scale={showAltitude ? [3.0, 1.12, 1] : [1.7, 0.85, 1]}>
              <spriteMaterial map={textures[i]} transparent depthWrite={false} />
            </sprite>
          </group>
        )
      })}
    </group>
  )
}
