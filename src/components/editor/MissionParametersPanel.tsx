/**
 * Mission parameters panel (FlightLab Phase 8c): the scalar knobs of the
 * run itself, each committed through `param/set` so every edit is one undo
 * entry. Bounds mirror the validator's caps and are named in the labels
 * (`LIMITS`), so the panel never promises a value the gate will reject.
 * Sensor noise lives behind a details block: it matters, but rarely.
 */

import { LIMITS } from '@/types/generated/talon_core'
import type { EditorCommand, MissionConfig } from '@/utils/editor'
import { NumberField } from './NumberField'
import { Section } from './Section'

export interface MissionParametersPanelProps {
  /** The mission being edited. */
  config: MissionConfig
  /** Apply one edit. */
  dispatch: (cmd: EditorCommand) => void
}

/** Shared styling for the text and area inputs in this panel. */
const TEXT_INPUT =
  'w-full border border-grid bg-panel px-2 py-1 font-sans text-[11px] text-ink transition-colors focus-visible:border-signal'

/** Mono label above a control, matching the NumberField label treatment. */
function FieldLabel({ htmlFor, children }: { htmlFor: string; children: string }) {
  return (
    <label
      htmlFor={htmlFor}
      className="font-mono text-[9px] uppercase tracking-[0.15em] text-ink-faint"
    >
      {children}
    </label>
  )
}

export function MissionParametersPanel({ config, dispatch }: MissionParametersPanelProps) {
  return (
    <Section title="Mission parameters">
      <div className="flex flex-col gap-2.5">
        <div className="flex flex-col gap-1">
          <FieldLabel htmlFor="mission-label">Label</FieldLabel>
          <input
            id="mission-label"
            type="text"
            value={config.label}
            onChange={(e) => dispatch({ type: 'param/set', key: 'label', value: e.target.value })}
            className={TEXT_INPUT}
          />
        </div>

        <div className="flex flex-col gap-1">
          <FieldLabel htmlFor="mission-blurb">Blurb</FieldLabel>
          <textarea
            id="mission-blurb"
            rows={2}
            value={config.blurb}
            onChange={(e) => dispatch({ type: 'param/set', key: 'blurb', value: e.target.value })}
            className={`${TEXT_INPUT} resize-y leading-relaxed`}
          />
        </div>

        <div className="grid grid-cols-2 gap-2">
          <NumberField
            label={`Duration s (max ${LIMITS.max_duration_s})`}
            value={config.duration_s}
            suffix="s"
            min={0.005}
            max={LIMITS.max_duration_s}
            step={5}
            onCommit={(value) => dispatch({ type: 'param/set', key: 'duration_s', value })}
          />
          <NumberField
            label={`Takeoff altitude m (max ${LIMITS.max_takeoff_altitude_m})`}
            value={config.takeoff_altitude}
            suffix="m"
            min={0.005}
            max={LIMITS.max_takeoff_altitude_m}
            onCommit={(value) => dispatch({ type: 'param/set', key: 'takeoff_altitude', value })}
          />
          <NumberField
            label="Battery SOC (0 to 1)"
            value={config.battery_initial_soc}
            suffix="frac"
            min={0}
            max={LIMITS.max_battery_soc}
            step={0.05}
            onCommit={(value) =>
              dispatch({ type: 'param/set', key: 'battery_initial_soc', value })
            }
          />
          <NumberField
            label={`Cruise hold s (max ${LIMITS.max_cruise_hold_s})`}
            value={config.cruise_hold_s}
            suffix="s"
            min={0}
            max={LIMITS.max_cruise_hold_s}
            step={0.5}
            onCommit={(value) => dispatch({ type: 'param/set', key: 'cruise_hold_s', value })}
          />
          <NumberField
            label="Seed (whole number)"
            value={config.seed}
            min={0}
            max={Number.MAX_SAFE_INTEGER}
            integer
            onCommit={(value) => dispatch({ type: 'param/set', key: 'seed', value })}
          />
          <div className="flex flex-col justify-end pb-1">
            <label className="flex cursor-pointer items-center gap-2 text-[11px] text-ink-dim transition-colors hover:text-ink">
              <input
                type="checkbox"
                checked={config.auto_arm}
                onChange={(e) =>
                  dispatch({ type: 'param/set', key: 'auto_arm', value: e.target.checked })
                }
                className="h-3 w-3 accent-signal"
              />
              Auto arm and take off
            </label>
          </div>
        </div>

        <details className="border border-grid bg-panel-2/50">
          <summary className="cursor-pointer px-2 py-1.5 font-mono text-[9px] uppercase tracking-[0.15em] text-ink-faint transition-colors hover:text-ink">
            Advanced: GPS noise
          </summary>
          <div className="grid grid-cols-2 gap-2 px-2 pb-2 pt-1">
            <NumberField
              label={`Horizontal m (max ${LIMITS.max_gps_position_noise_m})`}
              value={config.gps_position_noise}
              suffix="m"
              min={0}
              max={LIMITS.max_gps_position_noise_m}
              step={0.1}
              onCommit={(value) =>
                dispatch({ type: 'param/set', key: 'gps_position_noise', value })
              }
            />
            <NumberField
              label={`Vertical m (max ${LIMITS.max_gps_position_noise_m})`}
              value={config.gps_position_noise_vertical}
              suffix="m"
              min={0}
              max={LIMITS.max_gps_position_noise_m}
              step={0.1}
              onCommit={(value) =>
                dispatch({ type: 'param/set', key: 'gps_position_noise_vertical', value })
              }
            />
            <NumberField
              label={`Velocity m/s (max ${LIMITS.max_gps_velocity_noise_mps})`}
              value={config.gps_velocity_noise}
              suffix="m/s"
              min={0}
              max={LIMITS.max_gps_velocity_noise_mps}
              step={0.01}
              onCommit={(value) =>
                dispatch({ type: 'param/set', key: 'gps_velocity_noise', value })
              }
            />
          </div>
        </details>
      </div>
    </Section>
  )
}
