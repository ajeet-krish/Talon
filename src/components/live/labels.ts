/**
 * Canvas-rendered label sprites.
 *
 * The scene labels waypoints and keep-out zones with text. A DOM overlay would
 * be the obvious route, but it has to be projected and depth-tested by hand
 * every frame; a sprite is a texture on a quad, so the renderer does that work.
 * Rendering the text to a canvas keeps the app offline-safe: no font fetch, no
 * HTML-in-WebGL.
 */

import * as THREE from 'three'

/** One line of a label. */
export interface LabelLine {
  text: string
  /** Font size in canvas pixels. */
  size: number
  color: string
}

const CANVAS_W = 256
const CANVAS_H = 128

/** Render lines into a texture, centred, with the first line largest by default. */
export function labelTexture(lines: LabelLine[]): THREE.CanvasTexture {
  const canvas = document.createElement('canvas')
  canvas.width = CANVAS_W
  canvas.height = CANVAS_H
  const ctx = canvas.getContext('2d')
  if (ctx) {
    ctx.textAlign = 'center'
    ctx.textBaseline = 'middle'
    // Stack the lines around the vertical centre so a one-line label sits
    // centred and a two-line label does not drift upward.
    const lineH = CANVAS_H / (lines.length + 1)
    lines.forEach((line, i) => {
      ctx.font = `600 ${line.size}px "IBM Plex Mono", monospace`
      ctx.fillStyle = line.color
      ctx.fillText(line.text, CANVAS_W / 2, lineH * (i + 1))
    })
  }
  const tex = new THREE.CanvasTexture(canvas)
  tex.colorSpace = THREE.SRGBColorSpace
  return tex
}
