/**
 * The altitude ladder: a graduated vertical scale standing at the pad.
 *
 * Altitude is the one quantity a perspective camera can not read. The vehicle
 * carries its own plumb line and shadow, but those say "above there", not "ten
 * metres": an absolute statement needs something standing still to measure
 * against. So the pad gets a ruler.
 *
 * The rungs are a regular measurement grid sized from the mission's own scale,
 * and the rung at an altitude the mission actually flies is drawn longer, in
 * the signal colour, and glows. The ladder therefore states the plan and the
 * grid at once, and it is the only element in this scene that states a number.
 *
 * Everything here is mission-derived and static, so it costs no per-frame work:
 * it is placed once, beside the launch point, when the scenario is chosen.
 */

import { useEffect, useMemo } from 'react'
import type { LiveConfig } from '@/live/types'
import { nedToScene } from '@/utils/interpolation'
import { altitudeLadder, missionAltitudes } from '@/live/view'
import { labelTexture } from './labels'

/** Faint steel for the measuring grid. */
const RUNGS_COLOR = '#3d5170'
const AXIS_COLOR = '#2b3a52'
const LABEL_COLOR = '#8b9bb4'
const HEADER_COLOR = '#5f6f88'
/** Signal cyan: reserved for the altitude the mission holds. */
const FLIGHT_LEVEL_COLOR = '#26b8d9'

export function LiveAltitude({ cfg }: { cfg: LiveConfig }) {
  const ticks = useMemo(() => altitudeLadder(cfg), [cfg])
  const top = useMemo(() => missionAltitudes(cfg).reduce((m, a) => Math.max(m, a), 0), [cfg])

  // Bars and labels scale with the mission so the ladder reads at any frame:
  // a 10 m mission gets metre-scale rungs, a 100 m one gets ten-metre rungs.
  const bar = Math.max(0.06, top * 0.02)
  const offset = Math.max(1.4, top * 0.22)

  const textures = useMemo(
    () => ({
      header: labelTexture([
        { text: 'ALTITUDE', size: 26, color: LABEL_COLOR },
        { text: 'm above pad', size: 20, color: HEADER_COLOR },
      ]),
      rungs: ticks.map((tick) =>
        labelTexture([
          {
            text: String(tick.altitude),
            size: 34,
            color: tick.flightLevel ? FLIGHT_LEVEL_COLOR : LABEL_COLOR,
          },
        ]),
      ),
    }),
    [ticks],
  )

  useEffect(
    () => () => {
      textures.header.dispose()
      for (const tex of textures.rungs) tex.dispose()
    },
    [textures],
  )

  if (ticks.length === 0) return null

  const [homeX, , homeZ] = nedToScene(cfg.start_position_ned)

  return (
    <group position={[homeX + offset, 0, homeZ]}>
      {/* The spine: the datum the rungs hang off. */}
      <mesh position={[0, top / 2, 0]}>
        <cylinderGeometry args={[bar * 0.5, bar * 0.5, top, 6]} />
        <meshBasicMaterial color={AXIS_COLOR} />
      </mesh>
      {ticks.map((tick, i) => {
        const length = tick.flightLevel ? 1.3 : 0.8
        const thickness = tick.flightLevel ? bar * 1.5 : bar
        return (
          <group key={tick.altitude}>
            <mesh position={[length / 2, tick.altitude, 0]}>
              <boxGeometry args={[length, thickness, thickness]} />
              <meshBasicMaterial
                color={tick.flightLevel ? FLIGHT_LEVEL_COLOR : RUNGS_COLOR}
                transparent
                opacity={tick.flightLevel ? 1 : 0.7}
              />
            </mesh>
            <sprite position={[length + 1.1, tick.altitude, 0]} scale={[2.2, 1.1, 1]}>
              <spriteMaterial map={textures.rungs[i]} transparent depthWrite={false} />
            </sprite>
          </group>
        )
      })}
      <sprite position={[1.0, top + 1.8, 0]} scale={[3.2, 1.6, 1]}>
        <spriteMaterial map={textures.header} transparent depthWrite={false} />
      </sprite>
    </group>
  )
}
