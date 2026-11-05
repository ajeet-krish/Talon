/**
 * Minimal ambient shims so the node-only test helpers typecheck without pulling
 * the full @types/node package into the browser build. Only the few functions
 * those helpers use are declared.
 */

declare module 'node:fs' {
  export function readFileSync(path: string, encoding: 'utf8'): string
  export function existsSync(path: string): boolean
  export function readdirSync(path: string): string[]
  export function statSync(path: string): { isDirectory(): boolean; isFile(): boolean }
}

declare module 'node:module' {
  export function createRequire(url: string): (id: string) => unknown
}

declare module 'node:path' {
  export function resolve(...paths: string[]): string
  export function join(...paths: string[]): string
  export function dirname(path: string): string
  export function relative(from: string, to: string): string
  export const sep: string
}
