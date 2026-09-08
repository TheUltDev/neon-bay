// Controller reading. Keyboard or gamepad, normalized to the same four numbers
// the physics crate takes -- and the same four numbers that go on the wire.

import type { Input } from './sim';

const KEY_ACTIONS = {
  ArrowUp: 'up',
  KeyW: 'up',
  ArrowDown: 'down',
  KeyS: 'down',
  ArrowLeft: 'left',
  KeyA: 'left',
  ArrowRight: 'right',
  KeyD: 'right',
  Space: 'handbrake',
} as const;

type Action = (typeof KEY_ACTIONS)[keyof typeof KEY_ACTIONS];

export class Controls {
  private held = new Set<Action>();
  private tapHandlers = new Map<string, () => void>();
  gamepadActive = false;

  constructor(target: EventTarget = window) {
    target.addEventListener('keydown', (e) => {
      const ev = e as KeyboardEvent;
      if (ev.repeat) return;
      const tap = this.tapHandlers.get(ev.code);
      if (tap && !isTyping(ev)) {
        tap();
        ev.preventDefault();
      }
      const a = KEY_ACTIONS[ev.code as keyof typeof KEY_ACTIONS];
      if (a && !isTyping(ev)) {
        this.held.add(a);
        ev.preventDefault();
      }
    });
    target.addEventListener('keyup', (e) => {
      const a = KEY_ACTIONS[(e as KeyboardEvent).code as keyof typeof KEY_ACTIONS];
      if (a) this.held.delete(a);
    });
    target.addEventListener('blur', () => this.held.clear());
  }

  onTap(code: string, fn: () => void) {
    this.tapHandlers.set(code, fn);
  }

  /**
   * `forwardSpeed` decides whether "down" means brake or reverse, which is why
   * the mapping lives on the client: whatever it decides is what gets sent, so
   * prediction and authority always agree on the meaning.
   */
  read(forwardSpeed: number): Input {
    const pad = this.readPad();
    if (pad) {
      this.gamepadActive = true;
      return pad;
    }
    this.gamepadActive = false;

    const steer = (this.held.has('left') ? 1 : 0) + (this.held.has('right') ? -1 : 0);
    let throttle = this.held.has('up') ? 1 : 0;
    let brake = 0;
    if (this.held.has('down')) {
      if (forwardSpeed > 1.5) brake = 1;
      else throttle = -1;
    }
    return { throttle, steer, brake, handbrake: this.held.has('handbrake') ? 1 : 0 };
  }

  private readPad(): Input | null {
    const pads = navigator.getGamepads?.() ?? [];
    for (const p of pads) {
      if (!p) continue;
      const ax = p.axes[0] ?? 0;
      const rt = p.buttons[7]?.value ?? 0;
      const lt = p.buttons[6]?.value ?? 0;
      const hb = (p.buttons[0]?.pressed ? 1 : 0) || (p.buttons[5]?.pressed ? 1 : 0);
      const active = Math.abs(ax) > 0.12 || rt > 0.02 || lt > 0.02 || hb > 0;
      if (!active) continue;
      return {
        throttle: rt,
        steer: -deadzone(ax, 0.1),
        brake: lt,
        handbrake: hb,
      };
    }
    return null;
  }
}

function deadzone(v: number, dz: number): number {
  const a = Math.abs(v);
  if (a < dz) return 0;
  return Math.sign(v) * ((a - dz) / (1 - dz));
}

/** The <input> types that actually swallow keystrokes as text. */
const TEXT_INPUTS = new Set([
  'text',
  'search',
  'url',
  'tel',
  'email',
  'password',
  'number',
  'date',
  'datetime-local',
  'month',
  'week',
  'time',
]);

/**
 * Whether the keystroke belongs to a text field rather than to the car. Only the
 * driver-name box qualifies: a range slider or a checkbox is focusable but is
 * not a text field, and treating it as one left the controls dead until you
 * clicked away from the latency slider.
 */
function isTyping(e: KeyboardEvent): boolean {
  const t = e.target as HTMLElement | null;
  if (!t) return false;
  if (t.isContentEditable || t.tagName === 'TEXTAREA') return true;
  return t.tagName === 'INPUT' && TEXT_INPUTS.has((t as HTMLInputElement).type);
}
