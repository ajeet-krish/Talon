/**
 * Minimal ambient shims so the node-only test helpers typecheck without pulling
 * the full @types/node package into the browser build. Only the few functions
 * those helpers use are declared.
 */

declare module 'node:fs' {
  export function readFileSync(path: string, encoding: 'utf8'): string
}

declare module 'node:module' {
  export function createRequire(url: string): (id: string) => unknown
}
