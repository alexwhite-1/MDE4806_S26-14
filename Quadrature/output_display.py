"""Author: Alex White, 2/8/2026
The purpose of this file is as a live display for quadrature output (top) and associated angles (below).
Intended for use at expo or other demos, necessarily will need to be run at a much slower speed than the
actual hardware in order to be visually interpretable. Intended to be used along with python testing scripts,
which can be found in the other code in the folder.

Usage:
  Normal mode (subprocess script):
    python output_display.py

  Live angle mode (reads from shared temp file written by another script):
    python output_display.py --live

  Live angle mode, dual axis:
    python output_display.py --live --axes 2

  The feeding script must write angle lines to the file defined by LIVE_DATA_FILE.
  Single axis:  <angle>              e.g.  45.231
  Dual axis:    <angle1>,<angle2>    e.g.  45.231,120.005
"""
import subprocess
import threading
import tkinter as tk
from collections import deque
import sys
import os
import tempfile
import time


SCRIPT           = 'quad_out_script1.py'
SAMPLES          = 150
REDRAW_TIME      = 100   # ms
CPR              = 4096
ANGLE_RESOLUTION = 360.0 / (4 * CPR)
LABEL_BAR_H      = 40
POLL_INTERVAL    = 20    # ms — how often main thread checks for new data

# Shared file that the feeding script appends lines to.
# Both scripts must use the same path.
LIVE_DATA_FILE = os.path.join(tempfile.gettempdir(), 'quad_live_angles.txt')


# ---------------------------------------------------------------------------
# Quadrature helpers — matched to quad_out_script1.py
# ---------------------------------------------------------------------------

def quantize_angle_to_cpr(angle_deg: float) -> float:
    return round(angle_deg / ANGLE_RESOLUTION) * ANGLE_RESOLUTION

def quadrature_pattern_from_angle(angle_deg: float) -> tuple[int, int]:
    position = int((angle_deg % 360.0) / 360.0 * (4 * CPR)) % (4 * CPR)
    state = position % 4
    patterns = [(0, 0), (1, 0), (1, 1), (0, 1)]
    return patterns[state]

def angle_to_quadrature(angle_deg: float) -> tuple[int, int]:
    return quadrature_pattern_from_angle(quantize_angle_to_cpr(angle_deg) % 360.0)


# ---------------------------------------------------------------------------
# Main display
# ---------------------------------------------------------------------------

class LiveDisplay(tk.Tk):
    def __init__(self, live_mode: bool = False, num_axes: int = 1):
        super().__init__()
        self.live_mode = live_mode
        self.num_axes  = num_axes

        self.title('Quadrature Output Live Display' + (' [LIVE]' if live_mode else ''))
        self.resizable(False, False)

        self.canvas_w = 1000
        self.canvas_h = 500

        self.canvas = tk.Canvas(self, width=self.canvas_w, height=self.canvas_h, bg='white')
        self.canvas.pack(fill='none', expand=False, padx=8, pady=(8, 0))

        self.angle_frame = tk.Frame(self, height=LABEL_BAR_H)
        self.angle_frame.pack(fill='x', padx=8, pady=(0, 8))
        self.angle_frame.pack_propagate(False)

        self.angle1_label = tk.Label(self.angle_frame, text='Axis1: --.-°', font=('Consolas', 20))
        self.angle1_label.pack(side='left', padx=8)
        self.angle2_label = tk.Label(self.angle_frame, text='Axis2: --.-°', font=('Consolas', 20))
        self.angle2_label.pack(side='left', padx=8)

        mode_text  = 'MODE: LIVE ANGLE INPUT' if live_mode else 'MODE: SCRIPT'
        mode_color = '#0055cc' if live_mode else '#333333'
        tk.Label(self.angle_frame, text=mode_text, font=('Consolas', 14), fg=mode_color).pack(side='right', padx=16)

        self.update_idletasks()
        self.geometry(f'{self.canvas_w + 16}x{self.canvas_h + LABEL_BAR_H + 24}')

        self.a1_buf = deque([0] * SAMPLES, maxlen=SAMPLES)
        self.b1_buf = deque([0] * SAMPLES, maxlen=SAMPLES)
        self.a2_buf = deque([0] * SAMPLES, maxlen=SAMPLES)
        self.b2_buf = deque([0] * SAMPLES, maxlen=SAMPLES)

        self.index   = 0
        self.running = True
        self.proc    = None

        if self.live_mode:
            self._init_live_file()
            self.after(POLL_INTERVAL, self._poll_live_file)
        else:
            self._start_script()

        self.after(REDRAW_TIME, self.redraw)

    # ------------------------------------------------------------------
    # Mode A: subprocess script
    # ------------------------------------------------------------------

    def _start_script(self):
        try:
            self.proc = subprocess.Popen(
                [sys.executable, SCRIPT],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )
        except Exception as e:
            tk.messagebox.showerror('Error', f'Failed to start {SCRIPT}: {e}')
            self.destroy()
            return

        def reader():
            for raw in self.proc.stdout:
                if not self.running:
                    break
                line = raw.strip()
                if not line:
                    continue
                parts = line.split(',')
                if self.num_axes == 1 and len(parts) == 7:
                    self.num_axes = 2
                elif self.num_axes == 2 and len(parts) == 4:
                    self.num_axes = 1
                try:
                    if len(parts) == 4:
                        a1, b1 = int(parts[0]), int(parts[1])
                        idx = int(parts[2]); angle1 = float(parts[3])
                        self.a1_buf.append(a1); self.b1_buf.append(b1)
                        self.index = idx
                        self.after(0, self.angle1_label.config, {'text': f'Axis1: {angle1:.3f}°'})
                    elif len(parts) == 7:
                        a1, b1 = int(parts[0]), int(parts[1])
                        a2, b2 = int(parts[2]), int(parts[3])
                        idx = int(parts[4])
                        angle1, angle2 = float(parts[5]), float(parts[6])
                        self.a1_buf.append(a1); self.b1_buf.append(b1)
                        self.a2_buf.append(a2); self.b2_buf.append(b2)
                        self.index = idx
                        self.after(0, self.angle1_label.config, {'text': f'Axis1: {angle1:.3f}°'})
                        self.after(0, self.angle2_label.config, {'text': f'Axis2: {angle2:.3f}°'})
                except Exception:
                    continue

        threading.Thread(target=reader, daemon=True).start()

    # ------------------------------------------------------------------
    # Mode B: live file polling — no stdin, no blocking
    #
    # The feeding script appends lines to LIVE_DATA_FILE.
    # This display tracks a byte offset and reads only new lines each poll,
    # entirely on the main thread via after() — no threads needed.
    # ------------------------------------------------------------------

    def _init_live_file(self):
        # Truncate/create the file so we start clean
        with open(LIVE_DATA_FILE, 'w') as f:
            pass
        self._live_file_offset = 0
        print(f'[live mode] reading from: {LIVE_DATA_FILE}')

    def _poll_live_file(self):
        try:
            with open(LIVE_DATA_FILE, 'r') as f:
                f.seek(self._live_file_offset)
                new_data = f.read()
                self._live_file_offset = f.tell()
        except Exception:
            if self.running:
                self.after(POLL_INTERVAL, self._poll_live_file)
            return

        for raw in new_data.splitlines():
            line = raw.strip()
            if not line:
                continue
            parts = line.split(',')
            try:
                if len(parts) == 1:
                    angle1 = float(parts[0])
                    a1, b1 = angle_to_quadrature(angle1)
                    self.a1_buf.append(a1); self.b1_buf.append(b1)
                    self.num_axes = 1
                    self.angle1_label.config(text=f'Axis1: {angle1:.3f}°')
                elif len(parts) == 2:
                    angle1, angle2 = float(parts[0]), float(parts[1])
                    a1, b1 = angle_to_quadrature(angle1)
                    a2, b2 = angle_to_quadrature(angle2)
                    self.a1_buf.append(a1); self.b1_buf.append(b1)
                    self.a2_buf.append(a2); self.b2_buf.append(b2)
                    self.num_axes = 2
                    self.angle1_label.config(text=f'Axis1: {angle1:.3f}°')
                    self.angle2_label.config(text=f'Axis2: {angle2:.3f}°')
            except (ValueError, IndexError):
                continue

        if self.running:
            self.after(POLL_INTERVAL, self._poll_live_file)

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def redraw(self):
        self.canvas.delete('all')

        num_rows = self.num_axes * 2
        row_h  = max(40, self.canvas_h // num_rows)
        col_w  = max(1,  self.canvas_w // SAMPLES)

        buffers = [(self.a1_buf, 'A1'), (self.b1_buf, 'B1')]
        if self.num_axes == 2:
            buffers.extend([(self.a2_buf, 'A2'), (self.b2_buf, 'B2')])

        for i, (buf, label) in enumerate(buffers):
            y0     = i * row_h
            y_high = y0 + 10
            y_low  = y0 + row_h - 10

            self.canvas.create_text(8, (y_high + y_low) / 2, anchor='w',
                                    text=label, font=('Consolas', 16))

            snapshot = list(buf)
            coords = []
            x = self.canvas_w - col_w * len(snapshot)
            for v in snapshot:
                y = y_high if v else y_low
                coords.extend([x, y])
                x += col_w

            if len(coords) >= 4:
                self.canvas.create_line(coords, fill='green', width=2, smooth=False)

            self.canvas.create_line(60, y0 + row_h, self.canvas_w, y0 + row_h,
                                    fill='black', width=1)

        if not self.live_mode:
            idx_color = 'red' if self.index else 'lightgrey'
            self.canvas.create_oval(self.canvas_w - 36, 6, self.canvas_w - 10, 30,
                                    fill=idx_color, outline='black')
            self.canvas.create_text(self.canvas_w - 60, 18, text='INDEX', font=('Consolas', 10))

        if self.running:
            self.after(REDRAW_TIME, self.redraw)

    def on_close(self):
        self.running = False
        try:
            if self.proc and self.proc.poll() is None:
                self.proc.terminate()
        except Exception:
            pass
        self.destroy()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    live_mode = '--live' in sys.argv
    num_axes  = 1
    if '--axes' in sys.argv:
        try:
            num_axes = int(sys.argv[sys.argv.index('--axes') + 1])
        except (IndexError, ValueError):
            pass

    app = LiveDisplay(live_mode=live_mode, num_axes=num_axes)
    app.protocol('WM_DELETE_WINDOW', app.on_close)
    app.mainloop()

if __name__ == '__main__':
    main()