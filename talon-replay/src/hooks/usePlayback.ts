import { useFrame } from '@react-three/fiber'
import { useFlightStore } from '@/store/useFlightStore'

/**
 * Drive playback from the render loop. Must be called inside a <Canvas>
 * (it uses useFrame). Advances the store clock by real frame time scaled
 * by the playback speed; the store clamps to the scenario duration and
 * pauses at the end.
 *
 * Playback never autostarts: isPlaying begins false, so nothing moves
 * until the user presses play (respects reduced-motion preferences).
 */
export function usePlayback(): void {
  useFrame((_, delta) => {
    if (!useFlightStore.getState().isPlaying) return
    // Guard against tab-switch time jumps.
    const dt = Math.min(delta, 0.1) * useFlightStore.getState().playbackSpeed
    useFlightStore.getState().tick(dt)
  })
}
