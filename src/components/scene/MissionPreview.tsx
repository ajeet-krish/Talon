/**
 * 3D course preview (FlightLab Phase 8d-D): the chart's partner on the
 * mission page, framed identically beside it. It renders the current draft
 * only: the ground grid under the mission, the home marker, the numbered
 * route with altitudes, keep-out zones as amber spheres, and the selection
 * highlight the 2D chart draws. Every committed edit lands here immediately,
 * and a drag in flight tracks the pointer through the store's shared
 * transient, so the two surfaces never disagree about what is being
 * authored. It needs no telemetry: there is no run, no LiveBuffer, and no
 * drone.
 *
 * Read-only by construction: pointer input here orbits and zooms the camera
 * and nothing else, so no gesture in the preview can modify the mission.
 * The camera re-frames on the chart's triggers (mount, `documentEpoch`, and
 * this panel's own FIT button), never on a waypoint edit, so the frame the
 * user set survives an editing session. Wheel and drag stay on this canvas:
 * neither this panel nor OrbitControls registers a document-level
 * listener, so the chart's wheel is untouched and vice versa.
 *
 * Without WebGL the panel says so quietly and stops: the 2D chart and every
 * editing control carry on unaffected. A render failure that slips past the
 * probe lands in the same quiet state through an error boundary.
 */

import {
  Component,
  useEffect,
  useMemo,
  useRef,
  useState,
  type ReactNode,
  type RefObject,
} from 'react'
import { Canvas, useStore, useThree } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import * as THREE from 'three'
import type { Vec3 } from '@/types/flight'
import { useMissionStore } from '@/store/useMissionStore'
import { CHROME_BUTTON } from '@/components/ui/chrome'
import { withDragPreview, type MissionConfig } from '@/utils/editor'
import { nedToScene } from '@/utils/interpolation'
import { supportsWebGL } from '@/utils/webgl'
import { GroundPlane } from './GroundPlane'
import { ObstacleField } from './ObstacleField'
import { WaypointMarkers } from './WaypointMarkers'
import {
  PREVIEW_FOV_DEG,
  missionSceneBounds,
  previewCameraPose,
  previewGrid,
  type PreviewBounds,
} from './previewFraming'

/** The OrbitControls instance drei registers as the default controls. */
interface PreviewControls {
  target: THREE.Vector3
  update: () => void
  minDistance: number
  maxDistance: number
}

/** The preview's screen-reader summary: the mission and how to move the camera. */
function previewSummary(config: MissionConfig): string {
  const [north, east] = config.start_position_ned
  const parts = [`3D course preview. Home at ${north} meters north, ${east} meters east.`]
  if (config.waypoints.length === 0) {
    parts.push('No waypoints yet.')
  } else {
    parts.push(`${config.waypoints.length} waypoints:`)
    config.waypoints.forEach((wp, i) => {
      parts.push(`${i + 1} at ${-wp.position[2]} meters altitude.`)
    })
  }
  if (config.obstacles.length > 0) {
    parts.push(`${config.obstacles.length} keep-out zones:`)
    for (const ob of config.obstacles) {
      parts.push(`${ob.id} at radius ${ob.radius_m} meters.`)
    }
  }
  parts.push(
    'Drag to orbit and use the wheel to zoom. With the preview focused, ' +
      'arrow keys orbit and plus and minus zoom.',
  )
  return parts.join(' ')
}

/**
 * The quiet no-WebGL and stopped-rendering state: what happened, what it
 * does not affect, and nothing else. Direction, not mood.
 */
function PreviewFallback({ title, detail }: { title: string; detail: string }) {
  return (
    <div className="flex h-full w-full items-center justify-center p-6">
      <div className="max-w-[24rem] border border-grid bg-panel/85 px-4 py-3 text-center">
        <p className="font-mono text-[10px] tracking-[0.2em] text-ink-dim">{title}</p>
        <p className="mt-1.5 text-[11px] leading-relaxed text-ink-faint">{detail}</p>
      </div>
    </div>
  )
}

/**
 * Catches a WebGL failure the probe could not foresee (a context limit hit
 * later, a driver error during render) and swaps in the same quiet message,
 * so a broken preview never takes the page with it.
 */
class PreviewBoundary extends Component<
  { fallback: ReactNode; children: ReactNode },
  { failed: boolean }
> {
  state = { failed: false }

  static getDerivedStateFromError(): { failed: boolean } {
    return { failed: true }
  }

  render(): ReactNode {
    return this.state.failed ? this.props.fallback : this.props.children
  }
}

/** Single-line canvas sprite label for the home marker (the D9 label pattern). */
function makeHomeLabel(): THREE.CanvasTexture {
  const canvas = document.createElement('canvas')
  canvas.width = 128
  canvas.height = 64
  const ctx = canvas.getContext('2d')
  if (ctx) {
    ctx.font = '600 34px "IBM Plex Mono", monospace'
    ctx.fillStyle = '#dbe4f0'
    ctx.textAlign = 'center'
    ctx.textBaseline = 'middle'
    ctx.fillText('HOME', 64, 32)
  }
  const tex = new THREE.CanvasTexture(canvas)
  tex.colorSpace = THREE.SRGBColorSpace
  return tex
}

/** The chart's home diamond: a standing octahedron, ringed on the ground. */
function HomeMarker({ position }: { position: Vec3 }) {
  const [x, y, z] = position
  const label = useMemo(() => makeHomeLabel(), [])
  useEffect(() => () => label.dispose(), [label])
  return (
    <group position={[x, y, z]}>
      <mesh position={[0, 0.7, 0]} scale={[1, 1.4, 1]}>
        <octahedronGeometry args={[0.5, 0]} />
        <meshBasicMaterial color="#dbe4f0" />
      </mesh>
      {/* Four-segment ground ring: the chart's diamond footprint. */}
      <mesh position={[0, 0.03, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <ringGeometry args={[0.7, 0.85, 4]} />
        <meshBasicMaterial
          color="#8b9bb4"
          transparent
          opacity={0.6}
          side={THREE.DoubleSide}
          depthWrite={false}
        />
      </mesh>
      <sprite position={[0, 2, 0]} scale={[2.2, 1.1, 1]}>
        <spriteMaterial map={label} transparent depthWrite={false} />
      </sprite>
    </group>
  )
}

/**
 * The route through home and every waypoint, as a signal-cyan hairline. The
 * line is keyed on the drawn coordinates, so a drag swaps one small
 * geometry for another instead of mutating shared buffers, and the previous
 * geometry and material are disposed with it.
 */
function RouteLine({ points }: { points: readonly Vec3[] }) {
  const routeKey = points.map((p) => `${p[0]},${p[1]},${p[2]}`).join(';')
  const line = useMemo(() => {
    const geometry = new THREE.BufferGeometry().setFromPoints(
      points.map((p) => new THREE.Vector3(p[0], p[1], p[2])),
    )
    const material = new THREE.LineBasicMaterial({ color: '#26b8d9' })
    return new THREE.Line(geometry, material)
    // routeKey is the points' coordinates, so the cache and the key move
    // together; a selection change with equal coordinates reuses the line.
  }, [routeKey])
  useEffect(() => {
    return () => {
      line.geometry.dispose()
      line.material.dispose()
    }
  }, [line])
  return <primitive key={routeKey} object={line} />
}

interface PreviewRigProps {
  /** The frame to set on mount, on a document change, and on FIT. */
  bounds: PreviewBounds
  /** Store counter of document replacements, the chart's re-frame trigger. */
  documentEpoch: number
  /** Bumped by this panel's FIT button: the explicit re-frame trigger. */
  fitSignal: number
  /** The focusable wrapper the keyboard orbit listens on. */
  wrapperRef: RefObject<HTMLDivElement | null>
}

/**
 * The preview's camera: the one-time frame and the keyboard orbit. Every
 * re-frame reads `bounds` from the render its trigger belongs to and the
 * panel size fresh from the store, and lists neither as a dependency, so a
 * waypoint edit or a window resize never moves a camera the user has
 * already aimed.
 */
function PreviewRig({ bounds, documentEpoch, fitSignal, wrapperRef }: PreviewRigProps) {
  const store = useStore()
  const camera = useThree((s) => s.camera)
  const controls = useThree((s) => s.controls) as unknown as PreviewControls | null

  /**
   * Frame on mount, on `documentEpoch` (load, import, CLEAR), and on FIT:
   * the chart's three triggers, no others. The controls arrive one render
   * after the camera (drei mounts them), and that identity change re-runs
   * the fit once with the orbit target and the zoom clamps set.
   */
  useEffect(() => {
    const { size } = store.getState()
    const aspect = size.height > 0 ? size.width / size.height : 1
    const pose = previewCameraPose(bounds, aspect)
    camera.position.set(pose.position[0], pose.position[1], pose.position[2])
    camera.near = pose.near
    camera.far = pose.far
    camera.updateProjectionMatrix()
    camera.lookAt(pose.target[0], pose.target[1], pose.target[2])
    if (controls === null) return
    controls.target.set(pose.target[0], pose.target[1], pose.target[2])
    controls.minDistance = pose.minDistance
    controls.maxDistance = pose.maxDistance
    controls.update()
  }, [documentEpoch, fitSignal, controls])

  /**
   * Keyboard orbit for the focused wrapper: arrows rotate around the
   * target, plus and minus change the distance, both clamped to the pose's
   * limits. Registered on the wrapper element only, and only when the
   * wrapper itself holds focus (a focused corner control keeps its own
   * keys), so there is no document-level listener to fight the chart.
   */
  useEffect(() => {
    const el = wrapperRef.current
    if (el === null || controls === null) return
    const onKeyDown = (e: KeyboardEvent): void => {
      if (e.target !== el || e.metaKey || e.ctrlKey || e.altKey) return
      const offset = camera.position.clone().sub(controls.target)
      const spherical = new THREE.Spherical().setFromVector3(offset)
      if (!(spherical.radius > 0)) return
      switch (e.key) {
        case 'ArrowLeft':
          spherical.theta -= 0.12
          break
        case 'ArrowRight':
          spherical.theta += 0.12
          break
        case 'ArrowUp':
          spherical.phi -= 0.09
          break
        case 'ArrowDown':
          spherical.phi += 0.09
          break
        case '+':
        case '=':
          spherical.radius *= 0.88
          break
        case '-':
        case '_':
          spherical.radius *= 1.14
          break
        default:
          return
      }
      // The same polar limit OrbitControls holds, and the pose's zoom range.
      spherical.phi = Math.min(Math.max(spherical.phi, 0.05), Math.PI * 0.495)
      spherical.radius = Math.min(
        Math.max(spherical.radius, controls.minDistance),
        controls.maxDistance,
      )
      camera.position.copy(controls.target).add(offset.setFromSpherical(spherical))
      controls.update()
      e.preventDefault()
    }
    el.addEventListener('keydown', onKeyDown)
    return () => el.removeEventListener('keydown', onKeyDown)
  }, [wrapperRef, controls, camera])

  return null
}

/**
 * The 3D course preview panel. Fully store-driven: it subscribes to the
 * present config, the selection, the shared drag transient, and the
 * document epoch itself, so it takes no props and the view above it stays
 * free of the per-pointer-move writes.
 */
export function MissionPreview() {
  const config = useMissionStore((s) => s.history.present)
  const selected = useMissionStore((s) => s.selected)
  const dragPreview = useMissionStore((s) => s.dragPreview)
  const documentEpoch = useMissionStore((s) => s.documentEpoch)

  const wrapperRef = useRef<HTMLDivElement | null>(null)
  const [fitSignal, setFitSignal] = useState(0)
  /** Probed once per mount: no WebGL 2 means the quiet fallback, full stop. */
  const [webgl] = useState(() => supportsWebGL())
  /** Reduced motion: with it, orbit damping (camera inertia) is off. */
  const [reducedMotion] = useState(() =>
    typeof window !== 'undefined' && typeof window.matchMedia === 'function'
      ? window.matchMedia('(prefers-reduced-motion: reduce)').matches
      : false,
  )

  /**
   * The drawn draft: committed state with the shared drag transient
   * applied, exactly what the chart draws beside us. Idle renders get the
   * config by identity, so a selection change reuses every memo below.
   */
  const drawn = withDragPreview(config, dragPreview)
  const waypointsNed = useMemo(() => drawn.waypoints.map((wp) => wp.position), [drawn])
  const obstacles = drawn.obstacles
  const bounds = useMemo(() => missionSceneBounds(drawn), [drawn])
  const grid = useMemo(() => previewGrid(bounds), [bounds])
  const routePoints = useMemo(() => {
    const path: Vec3[] = [drawn.start_position_ned, ...drawn.waypoints.map((wp) => wp.position)]
    return path.map((ned) => {
      const [x, y, z] = nedToScene(ned)
      // Lifted off the grid so a ground-level leg never z-fights it.
      return [x, y + 0.06, z] as Vec3
    })
  }, [drawn])

  const waypointHighlight = selected?.kind === 'waypoint' ? selected.index : undefined
  const zoneHighlight = selected?.kind === 'obstacle' ? selected.index : undefined
  const summary = previewSummary(config)

  return (
    <div
      ref={wrapperRef}
      className="relative h-full w-full focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-[-2px] focus-visible:outline-signal"
      tabIndex={webgl ? 0 : undefined}
      role={webgl ? 'group' : undefined}
      aria-label={webgl ? summary : undefined}
    >
      {webgl ? (
        <PreviewBoundary
          fallback={
            <PreviewFallback
              title="3D PREVIEW STOPPED"
              detail="The course preview stopped rendering. The 2D chart and every editing control are unaffected."
            />
          }
        >
          <Canvas
            dpr={[1, 2]}
            gl={{ antialias: true }}
            camera={{ fov: PREVIEW_FOV_DEG, near: 0.1, far: 4000, position: [60, 40, 60] }}
            style={{ position: 'absolute', inset: 0 }}
          >
            <color attach="background" args={['#0e1526']} />
            {/* No lights: every preview material is unlit, the chart's flat
                instrument rendering rather than a lit scene. */}
            <OrbitControls
              makeDefault
              enableDamping={!reducedMotion}
              dampingFactor={0.08}
              maxPolarAngle={Math.PI * 0.495}
            />
            <PreviewRig
              bounds={bounds}
              documentEpoch={documentEpoch}
              fitSignal={fitSignal}
              wrapperRef={wrapperRef}
            />
            <GroundPlane size={grid.size} divisions={grid.divisions} position={grid.position} />
            {routePoints.length >= 2 && <RouteLine points={routePoints} />}
            <HomeMarker position={nedToScene(config.start_position_ned)} />
            <WaypointMarkers
              waypointsNed={waypointsNed}
              showAltitude
              highlight={waypointHighlight}
            />
            <ObstacleField obstacles={obstacles} highlight={zoneHighlight} />
          </Canvas>
        </PreviewBoundary>
      ) : (
        <PreviewFallback
          title="3D PREVIEW UNAVAILABLE"
          detail="This browser cannot create a WebGL context, so the course preview is off. The 2D chart and every editing control are unaffected."
        />
      )}

      {/* Quiet corner label: which of the two framed surfaces this is. */}
      <span
        aria-hidden="true"
        className="pointer-events-none absolute left-2 top-2 font-mono text-[9px] tracking-[0.15em] text-ink-faint"
      >
        3D PREVIEW
      </span>

      {/* The chart's FIT trigger, in the chart's corner: one explicit
          re-frame for this camera, and nothing else in the panel edits. */}
      {webgl && (
        <div
          role="group"
          aria-label="Preview camera controls"
          className="absolute bottom-2 right-2"
        >
          <button
            type="button"
            onClick={() => setFitSignal((n) => n + 1)}
            title="Frame the whole mission"
            className={CHROME_BUTTON}
          >
            FIT
          </button>
        </div>
      )}
    </div>
  )
}
