/**
 * Compact numeric formatting for instrument readouts. Printed precision is
 * proportional to magnitude so large values do not overflow their cell and
 * small ones keep resolution (EKF sigma vs altitude).
 */
export function formatMetric(value: number): string {
  if (!Number.isFinite(value)) return '--'
  const abs = Math.abs(value)
  if (abs >= 1000) return value.toFixed(0)
  if (abs >= 100) return value.toFixed(1)
  if (abs >= 10) return value.toFixed(2)
  if (abs === 0) return '0'
  return value.toFixed(3)
}
