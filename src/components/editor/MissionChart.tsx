/**
 * Store-wired mission chart (FlightLab Phase 8d-D). `MissionCanvas` stays
 * the chart engine with narrow props, including the shared drag transient;
 * this shell supplies that transient from `useMissionStore` so the chart
 * file itself never touches the store, which is what the Phase 8d-C
 * viewport contract asserts (zoom and pan are local state, and only the
 * sanctioned fit paths may move the frame).
 *
 * The subscriptions are granular: only `dragPreview` is read here, so a
 * pointer-move write re-renders the chart and the 3D preview that mirrors
 * it, never the inspector panels or the rest of the view, which select
 * `history.present` instead.
 */
import { useEffect } from 'react'
import { useMissionStore } from '@/store/useMissionStore'
import { MissionCanvas, type MissionCanvasProps } from './MissionCanvas'

/** Every chart prop except the transient, which the store supplies. */
type MissionChartProps = Omit<MissionCanvasProps, 'dragPreview' | 'onDragPreview'>

export function MissionChart(props: MissionChartProps) {
  const dragPreview = useMissionStore((s) => s.dragPreview)
  const setDragPreview = useMissionStore((s) => s.setDragPreview)

  /**
   * A gesture interrupted by unmount (a view switch mid-drag) must not
   * leave a phantom transient behind for the next visit to this page, so
   * the one path the chart's end handlers cannot reach clears it here.
   */
  useEffect(() => () => useMissionStore.getState().setDragPreview(null), [])

  return <MissionCanvas {...props} dragPreview={dragPreview} onDragPreview={setDragPreview} />
}
