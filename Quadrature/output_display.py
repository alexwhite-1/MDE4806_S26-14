"""Author: Alex White, 2/8/2026
The purpose of this file is as a live display for quadrature output (top) and associated angles (below).
Intended for use at expo or other demos, necessarily will need to be run at a much slower speed than the
actual hardware in order to be visually interpretable.

Usage:
  Normal mode (subprocess script):
    python output_display.py

  Live angle file mode:
    python output_display.py --live
    python output_display.py --live --axes 2

  Bluetooth SPP mode (ESP32 paired as a COM port on Windows):
    python output_display.py --bt COM3
    python output_display.py --bt COM3 --axes 2
    python output_display.py --bt COM3 --baud 115200

  ESP32 CSV format (full quadrature):
    Single axis:  A,B,Index,Angle          e.g.  1,0,0,45.2310
    Dual axis:    A1,B1,A2,B2,Index,Angle1,Angle2

  Dependencies:
    pip install pyserial
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
REDRAW_TIME      = 8    # ms (~60 fps visual refresh)
CPR              = 4096
ANGLE_RESOLUTION = 360.0 / (4 * CPR)
LABEL_BAR_H      = 40
POLL_INTERVAL    = 20    # ms — file polling interval (live mode only)
DEFAULT_BAUD     = 115200

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
    def __init__(self, live_mode: bool = False, bt_port: str = None,
                 baud: int = DEFAULT_BAUD, num_axes: int = 1):
        super().__init__()
        self.live_mode = live_mode
        self.bt_port   = bt_port
        self.baud      = baud
        self.num_axes  = num_axes

        # Determine display mode label
        if bt_port:
            mode_text  = f'MODE: BLUETOOTH  {bt_port}'
            mode_color = '#007700'
        elif live_mode:
            mode_text  = 'MODE: LIVE FILE INPUT'
            mode_color = '#0055cc'
        else:
            mode_text  = 'MODE: SCRIPT'
            mode_color = '#333333'

        self.title('Quadrature Output Live Display'
                   + (f' [BT {bt_port}]' if bt_port else ' [LIVE]' if live_mode else ''))
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

        tk.Label(self.angle_frame, text=mode_text, font=('Consolas', 14), fg=mode_color
                 ).pack(side='right', padx=16)

        self.update_idletasks()
        self.geometry(f'{self.canvas_w + 16}x{self.canvas_h + LABEL_BAR_H + 24}')

        self.a1_buf = deque([0] * SAMPLES, maxlen=SAMPLES)
        self.b1_buf = deque([0] * SAMPLES, maxlen=SAMPLES)
        self.a2_buf = deque([0] * SAMPLES, maxlen=SAMPLES)
        self.b2_buf = deque([0] * SAMPLES, maxlen=SAMPLES)

        self.index   = 0
        self.running = True
        self.proc    = None

        # Pending updates written by BT thread, drained on main thread
        self._pending = []
        self._pending_lock = threading.Lock()

        if self.bt_port:
            self._start_bt_reader()
            self.after(POLL_INTERVAL, self._drain_pending)
        elif self.live_mode:
            self._init_live_file()
            self.after(POLL_INTERVAL, self._poll_live_file)
        else:
            self._start_script()

        self.after(REDRAW_TIME, self.redraw)

    # ------------------------------------------------------------------
    # Shared: apply one parsed CSV line to buffers/labels.
    # Must be called on the main thread.
    # Accepts the full quadrature CSV format from the ESP32:
    #   single: A,B,Index,Angle
    #   dual:   A1,B1,A2,B2,Index,Angle1,Angle2
    # ------------------------------------------------------------------

    def _apply_line(self, line: str):
        parts = line.split(',')
        # Auto-detect axis count
        if len(parts) == 4:
            self.num_axes = 1
        elif len(parts) == 7:
            self.num_axes = 2
        else:
            return

        try:
            if self.num_axes == 1:
                a1, b1  = int(parts[0]), int(parts[1])
                idx     = int(parts[2])
                angle1  = float(parts[3])
                self.a1_buf.append(a1)
                self.b1_buf.append(b1)
                self.index = idx
                self.angle1_label.config(text=f'Axis1: {angle1:.3f}°')

            else:
                a1, b1  = int(parts[0]), int(parts[1])
                a2, b2  = int(parts[2]), int(parts[3])
                idx     = int(parts[4])
                angle1  = float(parts[5])
                angle2  = float(parts[6])
                self.a1_buf.append(a1); self.b1_buf.append(b1)
                self.a2_buf.append(a2); self.b2_buf.append(b2)
                self.index = idx
                self.angle1_label.config(text=f'Axis1: {angle1:.3f}°')
                self.angle2_label.config(text=f'Axis2: {angle2:.3f}°')

        except (ValueError, IndexError):
            pass

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
                if line:
                    self.after(0, self._apply_line, line)

        threading.Thread(target=reader, daemon=True).start()

    # ------------------------------------------------------------------
    # Mode B: live temp file polling
    # ------------------------------------------------------------------

    def _init_live_file(self):
        with open(LIVE_DATA_FILE, 'w'):
            pass
        self._live_offset = 0
        print(f'[live mode] reading from: {LIVE_DATA_FILE}')

    def _poll_live_file(self):
        try:
            with open(LIVE_DATA_FILE, 'r') as f:
                f.seek(self._live_offset)
                chunk = f.read()
                self._live_offset = f.tell()
            for raw in chunk.splitlines():
                line = raw.strip()
                if line:
                    self._apply_line(line)
        except Exception:
            pass
        if self.running:
            self.after(POLL_INTERVAL, self._poll_live_file)

    # ------------------------------------------------------------------
    # Mode C: Bluetooth SPP via pyserial
    #
    # The ESP32 paired over Classic BT appears as a COM port on Windows.
    # pyserial reads it like any serial device — no special BT library needed.
    #
    # The serial read runs in a background thread (readline() blocks).
    # Parsed lines are batched into self._pending under a lock.
    # _drain_pending() runs on the main thread via after() and applies them.
    # ------------------------------------------------------------------

    def _start_bt_reader(self):
        try:
            import serial
        except ImportError:
            tk.messagebox.showerror(
                'Missing dependency',
                'pyserial is required for Bluetooth mode.\n\nRun:  pip install pyserial'
            )
            self.destroy()
            return

        def reader():
            ser = None
            while self.running:
                # Retry connection loop — handles disconnect/reconnect
                try:
                    print(f'[bt] connecting to {self.bt_port} @ {self.baud} baud...')
                    ser = serial.Serial(
                        port=self.bt_port,
                        baudrate=self.baud,
                        timeout=2,          # 2 s read timeout so thread can check self.running
                    )
                    print(f'[bt] connected.')
                    while self.running:
                        raw = ser.readline()
                        if not raw:
                            continue        # timeout — loop back and check self.running
                        try:
                            line = raw.decode('utf-8', errors='replace').strip()
                        except Exception:
                            continue
                        if not line:
                            continue
                        with self._pending_lock:
                            self._pending.append(line)

                except Exception as e:
                    print(f'[bt] error: {e} — retrying in 2 s')
                    try:
                        if ser and ser.is_open:
                            ser.close()
                    except Exception:
                        pass
                    ser = None
                    for _ in range(20):     # 2 s back-off in 100 ms steps
                        if not self.running:
                            return
                        time.sleep(0.1)

            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception:
                pass

        threading.Thread(target=reader, daemon=True).start()

    def _drain_pending(self):
        """Drain BT lines onto the main thread and apply to buffers."""
        with self._pending_lock:
            lines, self._pending = self._pending, []
        for line in lines:
            self._apply_line(line)
        if self.running:
            self.after(POLL_INTERVAL, self._drain_pending)

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def redraw(self):
        self.canvas.delete('all')

        num_rows = self.num_axes * 2
        row_h = max(40, self.canvas_h // num_rows)
        col_w = max(1,  self.canvas_w // SAMPLES)

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
                self.canvas.create_line(coords, fill='blue', width=2, smooth=False)

            self.canvas.create_line(60, y0 + row_h, self.canvas_w, y0 + row_h,
                                    fill='black', width=1)

        # Index indicator
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
    args = sys.argv[1:]

    bt_port   = None
    live_mode = False
    num_axes  = 1
    baud      = DEFAULT_BAUD

    if '--bt' in args:
        try:
            bt_port = args[args.index('--bt') + 1]
        except IndexError:
            print('ERROR: --bt requires a COM port argument, e.g. --bt COM3')
            sys.exit(1)

    if '--live' in args:
        live_mode = True

    if '--axes' in args:
        try:
            num_axes = int(args[args.index('--axes') + 1])
        except (IndexError, ValueError):
            pass

    if '--baud' in args:
        try:
            baud = int(args[args.index('--baud') + 1])
        except (IndexError, ValueError):
            pass

    app = LiveDisplay(live_mode=live_mode, bt_port=bt_port,
                      baud=baud, num_axes=num_axes)
    app.protocol('WM_DELETE_WINDOW', app.on_close)
    app.mainloop()


if __name__ == '__main__':
    main()