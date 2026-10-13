import { useEffect } from 'react'
import { useFlightStore } from '@/store/useFlightStore'
import { FlightScene } from '@/components/scene/FlightScene'
import { Timeline } from '@/components/ui/Timeline'
import { Transport } from '@/components/ui/Transport'
import { ScenarioSelector } from '@/components/ui/ScenarioSelector'
import { DisplayToggles } from '@/components/ui/DisplayToggles'
import { HUD } from '@/components/ui/HUD'
import { Legend } from '@/components/ui/Legend'

/**
 * App shell: slim left instrument rail, full-bleed 3D canvas with the HUD
 * and legend overlaid, and a full-width bottom transport + phase-scrubber
 * bar. No hero, no landing page; the tool opens straight into the scene.
 */
export function App() {
  const activeId = useFlightStore((s) => s.activeScenario)
  const data = useFlightStore((s) => s.scenarios.get(s.activeScenario))
  const loading = useFlightStore((s) => s.loading)
  const error = useFlightStore((s) => s.error)
  const loadScenario = useFlightStore((s) => s.loadScenario)

  // Kick off the initial (lazy) fetch; other scenarios load on selection.
  useEffect(() => {
    void loadScenario('gps_outage')
  }, [loadScenario])

  return (
    <div className="flex h-screen w-screen flex-col overflow-hidden bg-panel text-ink">
      <div className="flex min-h-0 flex-1 flex-col md:flex-row">
        {/* Left instrument rail: scenario list + display toggles */}
        <aside className="flex shrink-0 flex-row items-stretch gap-4 overflow-x-auto border-b border-grid bg-panel px-2 py-2 md:w-56 md:flex-col md:overflow-visible md:border-b-0 md:border-r md:px-0 md:py-0">
          <div className="hidden flex-col gap-3 border-b border-grid px-3 py-4 md:flex">
            <div>
              <div className="font-mono text-[13px] font-semibold tracking-[0.2em] text-ink">
                TALON
              </div>
              <div className="font-mono text-[9px] tracking-[0.25em] text-ink-faint">
                FLIGHT REPLAY
              </div>
            </div>
            <div className="font-mono text-[9px] leading-relaxed text-ink-faint">
              Autopilot telemetry replay.
              <br />
              EKF state estimation, flight
              <br />
              state machine, NLGL guidance.
            </div>
          </div>
          <div className="md:border-b md:border-grid">
            <div className="px-3 pb-1 pt-2 font-mono text-[9px] tracking-[0.2em] text-ink-faint">
              SCENARIO
            </div>
            <ScenarioSelector />
          </div>
          <div className="md:pb-4">
            <div className="px-3 pb-1.5 pt-2 font-mono text-[9px] tracking-[0.2em] text-ink-faint">
              DISPLAY
            </div>
            <DisplayToggles />
          </div>
          <div className="hidden px-3 pt-3 font-mono text-[9px] leading-relaxed text-ink-faint md:block">
            TRAIL = TRUTH
            <br />
            ELLIPSOID = EKF 3&sigma;
          </div>
        </aside>

        {/* Full-bleed replay scene with overlays */}
        <main className="relative min-h-[280px] flex-1">
          <FlightScene />
          {data && <HUD data={data} />}
          {data && <Legend data={data} />}
          {!data && (
            <div className="absolute inset-0 flex items-center justify-center">
              <div className="rounded-[3px] border border-grid bg-panel/85 px-4 py-3 text-center">
                {error ? (
                  <>
                    <div className="font-mono text-[11px] text-[#D55E00]">
                      TELEMETRY LOAD FAILED
                    </div>
                    <div className="mt-1 text-[11px] text-ink-dim">{error}</div>
                  </>
                ) : (
                  <div className="font-mono text-[11px] tracking-wider text-ink-dim">
                    {loading ? 'LOADING TELEMETRY\u2026' : 'NO SCENARIO LOADED'}
                  </div>
                )}
              </div>
            </div>
          )}
        </main>
      </div>

      {/* Bottom transport + phase-segmented scrubber */}
      <footer className="flex shrink-0 items-stretch border-t border-grid bg-panel">
        <Transport />
        <div className="min-w-0 flex-1 px-4 py-2">
          {data ? (
            <Timeline key={activeId} data={data} />
          ) : (
            <div className="flex h-12 items-center font-mono text-[10px] text-ink-faint">
              AWAITING TELEMETRY
            </div>
          )}
        </div>
      </footer>
    </div>
  )
}
