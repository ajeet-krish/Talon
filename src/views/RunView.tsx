import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { useFlightStore } from '@/store/useFlightStore'
import { loadLiveConfigs } from '@/data/live'
import { loadEngine } from '@/live/engine'
import type { CoreModule } from '@/live/engine'
import { LiveSim } from '@/live/sim'
import { DT, DT_CONTROL } from '@/live/constants'
import { LiveBuffer, SAMPLE_STRIDE } from '@/live/stream'
import { Pacer, RATES, rateLabel } from '@/live/pacing'
import { bufferCapacity, readReadout } from '@/live/readout'
import type { LiveReadout } from '@/live/readout'
import { PHASE_COLORS } from '@/utils/colors'
import { LiveScene } from '@/components/live/LiveScene'
import { TelemetryRail } from '@/components/live/TelemetryRail'
import type { RunState } from '@/components/live/TelemetryRail'
import type { LiveConfig, LiveRunSummary } from '@/live/types'

type WasmState = 'loading' | 'ready' | 'unavailable'

/**
 * Most plant steps one frame may run.
 *
 * At the maximum rate the run is limited by the frame budget rather than by the
 * clock, and an unbounded budget would step a whole mission inside one frame and
 * freeze the page. 4000 plant steps is 20 s of flight, so even the longest
 * scenario finishes in a handful of frames while the tab stays interactive.
 */
const MAX_STEPS_PER_FRAME = 4000

/** Seconds between rail updates. The scene animates from the recorder, not React. */
const READOUT_INTERVAL_S = 0.1

/** Frames between phase-timeline refreshes (about 10 Hz at 60 fps). */
const PHASE_REFRESH_FRAMES = 6

function fmt(value: number, digits = 1): string {
  return Number.isFinite(value) ? value.toFixed(digits) : 'n/a'
}

const PHASE_LABELS: Record<string, string> = {
  idle: 'IDLE',
  arm: 'ARM',
  takeoff: 'TAKEOFF',
  cruise: 'CRUISE',
  waypoint: 'WAYPOINT',
  rtl: 'RTL',
  land: 'LAND',
  emergency: 'EMERGENCY',
}

/**
 * Live ground station. The Rust autopilot core (RK4 dynamics, Dryden wind,
 * cascaded PD, 15-state MEKF, NLGL guidance, waypoint mission, motor mixer, and
 * the SIL loop itself) is compiled to WebAssembly and stepped in this page at the
 * plant rate, then revealed at a pace you can watch.
 *
 * The run is real time by default with a speed control, because the point of a
 * ground station is to watch the vehicle fly and read the numbers as they change.
 * Pacing is a display concern: the core steps as fast as the frame allows and has
 * no idea it is being paced, so a paced run and a batch run are the same flight.
 * When a run finishes, its document is published to the replay, telemetry, and
 * metrics sections exactly like a recorded one.
 */
export function RunView() {
  const setView = useFlightStore((s) => s.setView)
  const setLiveRun = useFlightStore((s) => s.setLiveRun)

  const [configs, setConfigs] = useState<LiveConfig[] | null>(null)
  const [configError, setConfigError] = useState<string | null>(null)
  const [selectedId, setSelectedId] = useState<string | null>(null)
  const [wasm, setWasm] = useState<WasmState>('loading')
  const [wasmError, setWasmError] = useState<string | null>(null)
  const [runState, setRunState] = useState<RunState>('idle')
  const [rate, setRate] = useState<number>(1)
  const [readout, setReadout] = useState<LiveReadout | null>(null)
  const [summary, setSummary] = useState<LiveRunSummary | null>(null)
  /**
   * Bumped by Reset. The recorder is keyed on it so a reset gives the scene a
   * fresh, empty buffer instead of one holding the previous flight.
   */
  const [epoch, setEpoch] = useState(0)

  const engineRef = useRef<CoreModule | null>(null)
  const simRef = useRef<LiveSim | null>(null)
  const pacerRef = useRef<Pacer | null>(null)
  const rafRef = useRef<number | null>(null)
  const lastFrameRef = useRef<number | null>(null)
  const lastReadoutRef = useRef<number>(Number.NEGATIVE_INFINITY)
  const frameCountRef = useRef(0)
  const lastPhaseRef = useRef<string | null>(null)

  const selected = configs?.find((c) => c.id === selectedId) ?? null

  // One recorder per scenario per run, sized to the config's length. Recreating
  // it is what gives the scene a clean slate on a new scenario or a reset.
  const buffer = useMemo(
    () => (selected === null ? null : new LiveBuffer(bufferCapacity(selected, DT_CONTROL))),
    [selected, epoch],
  )

  // Scenario configs (one fetch per session; safe to refetch on remount).
  useEffect(() => {
    let cancelled = false
    loadLiveConfigs()
      .then((list) => {
        if (cancelled) return
        setConfigs(list)
        setSelectedId((current) => current ?? list[0]?.id ?? null)
      })
      .catch((err: unknown) => {
        if (!cancelled) setConfigError(err instanceof Error ? err.message : String(err))
      })
    return () => {
      cancelled = true
    }
  }, [])

  // WebAssembly kernel: loaded once. The sample layout is a hand-mirrored
  // contract with the Rust side, so a mismatch is caught here rather than
  // showing plausible but mislabelled numbers.
  useEffect(() => {
    let cancelled = false
    loadEngine()
      .then((mod) => {
        if (cancelled) return
        const stride = mod.sample_stride()
        if (stride !== SAMPLE_STRIDE) {
          setWasm('unavailable')
          setWasmError(
            `The wasm build streams ${stride} doubles per sample but this page decodes ${SAMPLE_STRIDE}. Rebuild with ./wasm/build.sh.`,
          )
          return
        }
        engineRef.current = mod
        setWasm('ready')
      })
      .catch((err: unknown) => {
        if (cancelled) return
        setWasm('unavailable')
        setWasmError(err instanceof Error ? err.message : String(err))
      })
    return () => {
      cancelled = true
    }
  }, [])

  const stopLoop = useCallback(() => {
    if (rafRef.current !== null) {
      cancelAnimationFrame(rafRef.current)
      rafRef.current = null
    }
  }, [])

  const disposeRun = useCallback(() => {
    stopLoop()
    simRef.current?.dispose()
    simRef.current = null
    pacerRef.current = null
    lastFrameRef.current = null
  }, [stopLoop])

  useEffect(() => disposeRun, [disposeRun])

  /**
   * Advance the native session to where the pacer says it should be, drain the
   * samples it recorded, and refresh the rail on a slow cadence.
   *
   * The step count is derived from the difference between the pacer's target and
   * the session's own clock, so the rounding of "how many plant steps fit in a
   * frame" never accumulates into a drift.
   */
  const startRun = useCallback(() => {
    const engine = engineRef.current
    if (selected === null || buffer === null || engine === null) return

    disposeRun()
    buffer.reset()
    setSummary(null)
    setReadout(null)
    lastReadoutRef.current = Number.NEGATIVE_INFINITY
    frameCountRef.current = 0
    lastPhaseRef.current = null

    const sim = LiveSim.create(selected, engine.Session)
    sim.prepare()
    const pacer = new Pacer(selected.duration_s, rate, 0)
    simRef.current = sim
    pacerRef.current = pacer
    setRunState('running')
    pacer.start(sim.time)

    const tick = (nowMs: number): void => {
      const active = simRef.current
      const activePacer = pacerRef.current
      if (active === null || activePacer === null) return

      const previous = lastFrameRef.current
      lastFrameRef.current = nowMs
      const deltaMs = previous === null ? 0 : nowMs - previous
      frameCountRef.current += 1

      if (activePacer.isRunning) {
        activePacer.tick(deltaMs)
        const needed = Math.round((activePacer.targetTime() - active.time) / DT)
        if (needed > 0) active.advanceMany(Math.min(needed, MAX_STEPS_PER_FRAME))
      }

      // Refresh the phase timeline before draining, so the samples appended this
      // frame are tagged with the timeline that produced them.
      const label = active.phase
      if (label !== lastPhaseRef.current || frameCountRef.current % PHASE_REFRESH_FRAMES === 1) {
        lastPhaseRef.current = label
        buffer.setPhases(active.phases())
      }

      if (active.sampleCount > buffer.count) {
        buffer.append(active.drain(buffer.count))
      }
      buffer.clock = active.time

      if (
        active.time - lastReadoutRef.current >= READOUT_INTERVAL_S ||
        active.time < lastReadoutRef.current
      ) {
        lastReadoutRef.current = active.time
        setReadout(readReadout(active, buffer, selected))
      }

      if (active.finished) {
        const result = active.finish()
        setReadout(readReadout(active, buffer, selected))
        setLiveRun(result.data, result.summary)
        setSummary(result.summary)
        activePacer.pause(active.time)
        active.dispose()
        simRef.current = null
        pacerRef.current = null
        rafRef.current = null
        setRunState('done')
        return
      }

      rafRef.current = requestAnimationFrame(tick)
    }

    rafRef.current = requestAnimationFrame(tick)
  }, [selected, buffer, rate, disposeRun, setLiveRun])

  const togglePause = useCallback(() => {
    const sim = simRef.current
    const pacer = pacerRef.current
    if (sim === null || pacer === null) return
    if (pacer.isRunning) {
      pacer.pause(sim.time)
      setRunState('paused')
    } else {
      pacer.start(sim.time)
      setRunState('running')
    }
  }, [])

  const resetRun = useCallback(() => {
    disposeRun()
    setRunState('idle')
    setReadout(null)
    setSummary(null)
    setEpoch((e) => e + 1)
  }, [disposeRun])

  const changeRate = useCallback(
    (next: number) => {
      setRate(next)
      const sim = simRef.current
      const pacer = pacerRef.current
      if (sim !== null && pacer !== null) pacer.setRate(next, sim.time)
    },
    [],
  )

  const selectScenario = useCallback(
    (id: string) => {
      if (id === selectedId) return
      disposeRun()
      setSelectedId(id)
      setRunState('idle')
      setReadout(null)
      setSummary(null)
    },
    [selectedId, disposeRun],
  )

  const canRun = wasm === 'ready' && selected !== null && runState !== 'running'
  const busy = runState === 'running'
  const phase = readout?.phase ?? 'idle'

  return (
    <div className="flex min-h-0 flex-1 flex-col">
      {/* Control strip */}
      <div className="flex shrink-0 flex-wrap items-center gap-x-4 gap-y-2 border-b border-grid bg-panel-2 px-3 py-2">
        <div className="flex items-baseline gap-2">
          <span className="font-mono text-[11px] font-semibold tracking-[0.2em] text-ink">
            LIVE RUN
          </span>
          <span
            className={`font-mono text-[9px] tracking-[0.18em] ${
              wasm === 'ready'
                ? 'text-signal'
                : wasm === 'loading'
                  ? 'text-ink-faint'
                  : 'text-amber-400'
            }`}
          >
            WASM {wasm.toUpperCase()}
          </span>
        </div>

        <div className="flex flex-wrap items-center gap-1">
          {(configs ?? []).map((cfg) => {
            const isSelected = cfg.id === selectedId
            return (
              <button
                key={cfg.id}
                type="button"
                disabled={busy}
                aria-pressed={isSelected}
                onClick={() => selectScenario(cfg.id)}
                className={`border px-2 py-1 font-mono text-[10px] tracking-wider transition-colors focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-signal disabled:opacity-50 ${
                  isSelected
                    ? 'border-signal bg-panel-3 text-signal'
                    : 'border-grid text-ink-dim hover:bg-panel-3'
                }`}
              >
                {cfg.label.toUpperCase()}
              </button>
            )
          })}
          {configs === null && configError === null && (
            <span className="font-mono text-[10px] text-ink-faint">LOADING CONFIGS</span>
          )}
        </div>

        <div className="flex items-center gap-1">
          <span className="font-mono text-[9px] tracking-[0.18em] text-ink-faint">RATE</span>
          {RATES.map((r) => (
            <button
              key={r}
              type="button"
              aria-pressed={rate === r}
              onClick={() => changeRate(r)}
              className={`border px-2 py-1 font-mono text-[10px] tabular-nums transition-colors ${
                rate === r
                  ? 'border-signal bg-panel-3 text-signal'
                  : 'border-grid text-ink-dim hover:bg-panel-3'
              }`}
            >
              {rateLabel(r)}
            </button>
          ))}
        </div>

        <div className="flex items-center gap-2">
          <button
            type="button"
            onClick={busy || runState === 'paused' ? togglePause : startRun}
            disabled={!canRun && runState !== 'paused'}
            className={`border px-3 py-1 font-mono text-[11px] tracking-[0.2em] transition-colors ${
              busy
                ? 'border-signal bg-panel-3 text-signal'
                : canRun || runState === 'paused'
                  ? 'border-signal text-signal hover:bg-panel-3'
                  : 'border-grid text-ink-faint'
            }`}
          >
            {busy ? 'PAUSE' : runState === 'paused' ? 'RESUME' : 'RUN'}
          </button>
          <button
            type="button"
            onClick={resetRun}
            disabled={runState === 'idle' && summary === null}
            className="border border-grid px-2 py-1 font-mono text-[10px] tracking-[0.2em] text-ink-dim transition-colors hover:bg-panel-3 disabled:opacity-40"
          >
            RESET
          </button>
        </div>

        <div className="ml-auto flex items-center gap-3">
          <span className="font-mono text-[10px] tabular-nums text-ink-dim">
            T+ {fmt(readout?.time ?? 0, 2)} s
          </span>
          <span className="flex items-center gap-1.5">
            <span
              className="inline-block h-2 w-2"
              style={{ backgroundColor: PHASE_COLORS[phase] }}
            />
            <span className="font-mono text-[10px] tracking-[0.15em] text-ink">
              {PHASE_LABELS[phase] ?? phase.toUpperCase()}
            </span>
          </span>
        </div>
      </div>

      {configError !== null && (
        <div className="shrink-0 border-b border-grid bg-panel-2 px-3 py-2 font-mono text-[10px] text-amber-400">
          LIVE CONFIGS UNAVAILABLE: {configError}
        </div>
      )}

      {wasm === 'unavailable' && (
        <div className="shrink-0 border-b border-grid bg-panel-3 px-3 py-2 text-[10px] leading-relaxed text-ink-dim">
          <span className="font-mono text-amber-400">KERNEL NOT LOADED.</span> The WebAssembly
          build was not found or does not match this page, so the live run is disabled. Recorded
          scenarios are unaffected. Build it with{' '}
          <span className="font-mono text-ink">./wasm/build.sh</span>.
          {wasmError !== null && (
            <>
              <br />
              <span className="font-mono text-ink-faint">{wasmError}</span>
            </>
          )}
        </div>
      )}

      {/* Scene and rail */}
      <div className="min-h-0 flex-1 overflow-y-auto xl:overflow-hidden">
        <div className="flex min-h-0 flex-col xl:h-full xl:flex-row">
          <div className="relative h-[54vh] min-h-0 w-full shrink-0 xl:h-auto xl:flex-1">
            {selected !== null && buffer !== null ? (
              <LiveScene cfg={selected} buffer={buffer} />
            ) : (
              <div className="flex h-full items-center justify-center bg-panel-2 font-mono text-[10px] tracking-[0.2em] text-ink-faint">
                SELECT A SCENARIO
              </div>
            )}

            {/* Overlay: what is being flown, and how to look around. */}
            <div className="pointer-events-none absolute left-3 top-3 flex flex-col gap-1">
              <span className="font-mono text-[11px] font-semibold tracking-[0.18em] text-ink">
                {selected?.label.toUpperCase() ?? 'NO SCENARIO'}
              </span>
              <span className="max-w-md font-mono text-[9px] leading-relaxed text-ink-faint">
                {selected?.blurb ?? ''}
              </span>
            </div>
            <div className="pointer-events-none absolute bottom-3 left-3 font-mono text-[9px] tracking-[0.18em] text-ink-faint">
              DRAG TO ORBIT / SCROLL TO ZOOM
            </div>
            {runState === 'idle' && readout === null && selected !== null && (
              <div className="pointer-events-none absolute inset-x-0 bottom-10 flex justify-center">
                <span className="border border-grid bg-panel/90 px-3 py-1.5 font-mono text-[10px] tracking-[0.18em] text-signal">
                  PRESS RUN TO FLY THE NATIVE CORE
                </span>
              </div>
            )}
          </div>

          <aside className="w-full shrink-0 border-t border-grid p-2 xl:flex xl:h-full xl:w-[380px] xl:flex-col xl:border-l xl:border-t-0">
            {selected !== null && buffer !== null ? (
              <TelemetryRail
                cfg={selected}
                buffer={buffer}
                readout={readout}
                runState={runState}
              />
            ) : (
              <div className="font-mono text-[10px] tracking-[0.18em] text-ink-faint">
                AWAITING SCENARIO
              </div>
            )}
          </aside>
        </div>
      </div>

      {summary !== null && (
        <section className="shrink-0 border-t border-grid bg-panel">
          <div className="flex items-center justify-between border-b border-grid px-3 py-1.5">
            <span className="font-mono text-[9px] tracking-[0.2em] text-ink-faint">LAST RUN</span>
            <button
              type="button"
              onClick={() => setView('replay')}
              className="font-mono text-[9px] tracking-[0.2em] text-signal hover:underline"
            >
              OPEN IN REPLAY
            </button>
          </div>
          <dl className="grid grid-cols-2 gap-px bg-grid sm:grid-cols-3 lg:grid-cols-6">
            {[
              {
                label: 'PHASES',
                value: summary.phases.map((p) => PHASE_LABELS[p] ?? p).join(' / '),
              },
              {
                label: 'WAYPOINTS',
                value: `${summary.waypoints_reached} / ${summary.waypoint_count}`,
              },
              { label: 'MISSION', value: summary.mission_complete ? 'COMPLETE' : 'INCOMPLETE' },
              { label: 'EKF RMSE', value: `${fmt(summary.ekf_position_rmse_m, 2)} m` },
              {
                label: 'MIN CLEARANCE',
                value:
                  summary.min_clearance_m === null
                    ? 'n/a'
                    : `${fmt(summary.min_clearance_m, 2)} m`,
              },
              {
                label: 'GPS FIXES',
                value: `${summary.gps_accepted} / ${summary.gps_accepted + summary.gps_gated}`,
              },
            ].map((tile) => (
              <div key={tile.label} className="bg-panel px-3 py-2">
                <dt className="font-mono text-[9px] tracking-[0.15em] text-ink-faint">
                  {tile.label}
                </dt>
                <dd
                  className={`mt-1 ${
                    tile.label === 'PHASES'
                      ? 'font-mono text-[10px] text-ink-dim'
                      : 'font-mono text-[11px] tabular-nums text-ink'
                  }`}
                >
                  {tile.value}
                </dd>
              </div>
            ))}
          </dl>
        </section>
      )}
    </div>
  )
}
