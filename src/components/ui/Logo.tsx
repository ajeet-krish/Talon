/**
 * The Talon FlightLab shield mark. Decorative by design: the TALON /
 * FLIGHTLAB wordmark beside it carries the name, so the image is hidden
 * from assistive technology to keep the header read once. Sizing and
 * spacing belong to the call site through className.
 */
export function Logo({ className }: { className?: string }) {
  return (
    <img
      src={`${import.meta.env.BASE_URL}logo.svg`}
      alt=""
      aria-hidden="true"
      className={className}
    />
  )
}
