import { useEffect } from 'react'
import { useFlightStore } from '@/store/useFlightStore'
import { Logo } from '@/components/ui/Logo'
import { ScenarioSelector } from '@/components/ui/ScenarioSelector'
import { DisplayToggles } from '@/components/ui/DisplayToggles'
import { ViewTabs } from '@/components/ui/ViewTabs'
import { Timeline } from '@/components/ui/Timeline'
import { Transport } from '@/components/ui/Transport'
import { ReplayView } from '@/views/ReplayView'
import { TelemetryView } from '@/views/TelemetryView'
import { MetricsView } from '@/views/MetricsView'
import { VideosView } from '@/views/VideosView'
import { RunView } from '@/views/RunView'

/**
 * Talon FlightLab shell: a header carrying the wordmark and the section tabs,
 * a slim left instrument rail (scenario list, plus display toggles on the
 * time-based sections), the active section filling the body, and the
 * transport and phase scrubber pinned to the bottom for the sections that
 * run on the replay clock. No hero, no landing page; the tool opens straight
 * into the scene.
 */
export function App() {
  const activeId = useFlightStore((s) => s.activeScenario)
  const activeView = useFlightStore((s) => s.activeView)
  const data = useFlightStore((s) => s.scenarios.get(s.activeScenario))
  const loadScenario = useFlightStore((s) => s.loadScenario)

  // Kick off the initial (lazy) fetch; other scenarios load on selection.
  useEffect(() => {
    void loadScenario('gps_outage')
  }, [loadScenario])

  // The replay and telemetry sections run on the shared clock, so they keep
  // the transport bar and the scene-layer toggles; metrics and videos do not.
  const timeBased = activeView === 'replay' || activeView === 'telemetry'

  return (
    <div className="flex h-screen w-screen flex-col overflow-hidden bg-panel text-ink">
      <header className="flex shrink-0 flex-wrap items-center gap-x-5 gap-y-1 border-b border-grid px-3 py-2">
        <div className="flex items-center gap-2">
          <Logo className="h-6 w-6 shrink-0" />
          <div className="flex items-baseline gap-2">
            <span className="font-mono text-[13px] font-semibold tracking-[0.2em] text-ink">
              TALON
            </span>
            <span className="font-mono text-[9px] tracking-[0.25em] text-ink-faint">
              FLIGHTLAB
            </span>
          </div>
        </div>
        <ViewTabs />
        <div className="ml-auto hidden font-mono text-[9px] tracking-wider text-ink-faint lg:block">
          AUTOPILOT TELEMETRY / EKF / STATE MACHINE / NLGL
        </div>
      </header>

      <div className="flex min-h-0 flex-1 flex-col md:flex-row">
        {/* Left instrument rail: scenario list, plus display toggles */}
        <aside className="flex shrink-0 flex-row items-stretch gap-4 overflow-x-auto border-b border-grid bg-panel px-2 py-2 md:w-56 md:flex-col md:overflow-visible md:border-b-0 md:border-r md:px-0 md:py-0">
          <div className="md:border-b md:border-grid">
            <div className="px-3 pb-1 pt-3 font-mono text-[9px] tracking-[0.2em] text-ink-faint">
              SCENARIO
            </div>
            <ScenarioSelector />
          </div>
          {timeBased && (
            <>
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
            </>
          )}
        </aside>

        {/* Active section */}
        <main className="relative flex min-h-0 min-w-0 flex-1 flex-col">
          {activeView === 'replay' && <ReplayView />}
          {activeView === 'telemetry' && <TelemetryView />}
          {activeView === 'metrics' && <MetricsView />}
          {activeView === 'videos' && <VideosView />}
          {activeView === 'run' && <RunView />}
        </main>
      </div>

      {/* Bottom transport + phase-segmented scrubber (time-based sections) */}
      {timeBased && (
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
      )}
    </div>
  )
}
