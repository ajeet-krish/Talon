import { useCallback, useEffect, useRef, useState } from 'react'
import { useFlightStore } from '@/store/useFlightStore'
import { loadLiveConfigs } from '@/data/live'
import { loadWasmModule } from '@/live/kernel'
import type { KernelHandle, TalonModule } from '@/live/kernel'
import { LiveSim } from '@/live/sim'
import type { LiveConfig, LiveRunSummary } from '@/live/types'

type WasmState = 'loading' | 'ready' | 'unavailable'
type RunState = 'idle' | 'running' | 'done'

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

function fmt(value: number, digits = 2): string {
  return Number.isFinite(value) ? value.toFixed(digits) : 'n/a'
}

/**
 * Live in-browser SIL. The same C++ autopilot kernel the recorded runs use
 * (dynamics, Dryden wind, cascaded PD, 15-state MEKF) is compiled to
 * WebAssembly and driven here by the TypeScript mission, guidance, motor, and
 * state-machine glue. Pick a scenario, run it, then open the result in the
 * replay, telemetry, and metrics sections like any recorded flight.
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
  const [progress, setProgress] = useState(0)
  const [summary, setSummary] = useState<LiveRunSummary | null>(null)

  const moduleRef = useRef<TalonModule | null>(null)
  const kernelRef = useRef<KernelHandle | null>(null)
  const rafRef = useRef<number | null>(null)

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

  // WASM kernel: loaded once; if it fails, Run is disabled with a clear note.
  useEffect(() => {
    let cancelled = false
    loadWasmModule()
      .then((mod) => {
        if (cancelled) return
        moduleRef.current = mod
        kernelRef.current = new mod.Kernel()
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

  useEffect(
    () => () => {
      if (rafRef.current !== null) cancelAnimationFrame(rafRef.current)
    },
    [],
  )

  const startRun = useCallback(() => {
    const cfg = configs?.find((c) => c.id === selectedId)
    const kernel = kernelRef.current
    if (cfg === undefined || kernel === null || runState === 'running') return

    const sim = new LiveSim(cfg, kernel)
    sim.prepare()
    setRunState('running')
    setProgress(0)
    setSummary(null)

    // Drive the 200 Hz loop from animation frames so the panel stays
    // responsive: run as many plant steps as fit in a ~12 ms budget per frame.
    const tick = () => {
      const deadline = performance.now() + 12
      let done = false
      while (performance.now() < deadline) {
        done = sim.advance()
        if (done) break
      }
      setProgress(sim.progress)
      if (done) {
        const result = sim.finish()
        setLiveRun(result.data, result.summary)
        setSummary(result.summary)
        setRunState('done')
        rafRef.current = null
        return
      }
      rafRef.current = requestAnimationFrame(tick)
    }
    rafRef.current = requestAnimationFrame(tick)
  }, [configs, selectedId, runState, setLiveRun])

  const selected = configs?.find((c) => c.id === selectedId) ?? null
  const canRun = wasm === 'ready' && selected !== null && runState !== 'running'

  return (
    <div className="min-h-0 flex-1 overflow-y-auto">
      <div className="mx-auto flex max-w-3xl flex-col gap-4 px-4 py-5">
        <div className="flex flex-wrap items-baseline justify-between gap-2">
          <div className="flex items-baseline gap-3">
            <h1 className="font-mono text-[12px] font-semibold tracking-[0.2em] text-ink">
              LIVE RUN
            </h1>
            <span className="font-mono text-[9px] tracking-[0.2em] text-ink-faint">
              C++ KERNEL / WEBSOCKET FREE / WASM
            </span>
          </div>
          <span
            className={`font-mono text-[9px] tracking-[0.2em] ${
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

        <p className="max-w-2xl text-[11px] leading-relaxed text-ink-dim">
          Fly a scenario here and now: the autopilot kernel (RK4 dynamics,
          Dryden turbulence, cascaded PD, 15-state MEKF) is compiled to
          WebAssembly and driven in the browser by the same mission, NLGL
          guidance, motor, sensor, and state-machine logic the recorded runs
          use. The result drops into the replay, telemetry, and metrics
          sections exactly like a recorded flight.
        </p>

        {configError !== null && (
          <div className="border-l-2 border-amber-400 bg-panel-2 px-3 py-2 font-mono text-[10px] text-amber-400">
            LIVE CONFIGS UNAVAILABLE: {configError}
          </div>
        )}

        {wasm === 'unavailable' && (
          <div className="border-l-2 border-amber-400 bg-panel-2 px-3 py-2 text-[10px] leading-relaxed text-ink-dim">
            <span className="font-mono text-amber-400">KERNEL NOT LOADED.</span>{' '}
            The WebAssembly build was not found, so Run is disabled. Recorded
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

        <section className="border border-grid">
          <div className="border-b border-grid px-3 py-1.5 font-mono text-[9px] tracking-[0.2em] text-ink-faint">
            SCENARIO
          </div>
          <div className="grid grid-cols-1 gap-px bg-grid sm:grid-cols-2">
            {(configs ?? []).map((cfg) => {
              const isSelected = cfg.id === selectedId
              return (
                <button
                  key={cfg.id}
                  type="button"
                  onClick={() => setSelectedId(cfg.id)}
                  className={`flex flex-col items-start gap-1 px-3 py-2.5 text-left transition-colors focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-[-2px] focus-visible:outline-signal ${
                    isSelected ? 'bg-panel-3' : 'bg-panel hover:bg-panel-2'
                  }`}
                >
                  <span className="flex w-full items-baseline justify-between gap-2">
                    <span
                      className={`font-mono text-[11px] font-semibold tracking-wider ${
                        isSelected ? 'text-signal' : 'text-ink-dim'
                      }`}
                    >
                      {cfg.label.toUpperCase()}
                    </span>
                    <span className="font-mono text-[9px] text-ink-faint">
                      {cfg.duration_s.toFixed(0)} s
                    </span>
                  </span>
                  <span className="text-[10px] leading-tight text-ink-faint">
                    {cfg.blurb}
                  </span>
                </button>
              )
            })}
            {configs === null && configError === null && (
              <div className="bg-panel px-3 py-3 font-mono text-[10px] text-ink-faint">
                LOADING CONFIGS
              </div>
            )}
          </div>
        </section>

        <section className="flex flex-wrap items-center gap-3 border border-grid px-3 py-3">
          <button
            type="button"
            onClick={startRun}
            disabled={!canRun}
            className={`border px-4 py-2 font-mono text-[11px] tracking-[0.2em] transition-colors ${
              canRun
                ? 'border-signal text-signal hover:bg-panel-2'
                : 'border-grid text-ink-faint'
            }`}
          >
            {runState === 'running' ? 'RUNNING' : 'RUN'}
          </button>

          <div className="min-w-40 flex-1">
            <div className="h-1 w-full bg-panel-3">
              <div
                className="h-1 bg-signal transition-[width] duration-75"
                style={{ width: `${Math.round(progress * 100)}%` }}
              />
            </div>
            <div className="mt-1.5 flex justify-between font-mono text-[9px] text-ink-faint">
              <span>
                {selected !== null
                  ? `${selected.duration_s.toFixed(0)} s AT 200 Hz (PLANT) / 100 Hz (CONTROL)`
                  : 'SELECT A SCENARIO'}
              </span>
              <span>{Math.round(progress * 100)}%</span>
            </div>
          </div>
        </section>

        {summary !== null && (
          <section className="border border-grid">
            <div className="flex items-center justify-between border-b border-grid px-3 py-1.5">
              <span className="font-mono text-[9px] tracking-[0.2em] text-ink-faint">
                LAST RUN
              </span>
              <button
                type="button"
                onClick={() => setView('replay')}
                className="font-mono text-[9px] tracking-[0.2em] text-signal hover:underline"
              >
                OPEN IN REPLAY
              </button>
            </div>
            <dl className="grid grid-cols-2 gap-px bg-grid sm:grid-cols-3">
              {[
                { label: 'PHASES', value: summary.phases.map((p) => PHASE_LABELS[p] ?? p).join('  \u203a  ') },
                {
                  label: 'WAYPOINTS',
                  value: `${summary.waypoints_reached} / ${summary.waypoint_count}`,
                },
                {
                  label: 'MISSION',
                  value: summary.mission_complete ? 'COMPLETE' : 'INCOMPLETE',
                },
                { label: 'EKF RMSE', value: `${fmt(summary.ekf_position_rmse_m)} m` },
                {
                  label: 'MIN CLEARANCE',
                  value:
                    summary.min_clearance_m === null
                      ? 'n/a'
                      : `${fmt(summary.min_clearance_m)} m`,
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
                    className={`mt-1 text-[11px] ${
                      tile.label === 'PHASES'
                        ? 'font-mono text-[10px] text-ink-dim'
                        : 'font-mono text-ink'
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
    </div>
  )
}
