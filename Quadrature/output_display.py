"""Live display for quadrature output (top) and associated angles (below).

Spawns `quad_out_script1.py` to receive simulated CSV data:
Format: Axis1A,Axis1B,Axis2A,Axis2B,Index,Angle1,Angle2

Usage: python output_display.py
"""
import subprocess
import threading
import tkinter as tk
from collections import deque
import time
import sys

SCRIPT = 'quad_out_script1.py'
SAMPLES = 400  # how many time columns to keep for waveform
NUM_AXES = None  # auto-detect from first CSV line


class LiveDisplay(tk.Tk):
	def __init__(self):
		super().__init__()
		self.title('Quadrature Output Live Display')
		self.resizable(False, False)

		self.canvas_w = 1200
		self.canvas_h = 360
		# allow resizing and expand canvas
		self.canvas = tk.Canvas(self, width=self.canvas_w, height=self.canvas_h, bg='white')
		self.canvas.pack(fill='both', expand=True, padx=8, pady=8)

		# labels for angles below
		self.angle_frame = tk.Frame(self)
		self.angle_frame.pack(fill='x', padx=8, pady=(0,8))
		self.angle1_label = tk.Label(self.angle_frame, text='Axis1: --.-°', font=('Consolas', 14))
		self.angle1_label.pack(side='left', padx=8)
		self.angle2_label = tk.Label(self.angle_frame, text='Axis2: --.-°', font=('Consolas', 14))
		self.angle2_label.pack(side='left', padx=8)

		# buffers for waveform: each holds last SAMPLES of 0/1
		self.a1_buf = deque([0]*SAMPLES, maxlen=SAMPLES)
		self.b1_buf = deque([0]*SAMPLES, maxlen=SAMPLES)
		self.a2_buf = deque([0]*SAMPLES, maxlen=SAMPLES)
		self.b2_buf = deque([0]*SAMPLES, maxlen=SAMPLES)

		# current index and num axes
		self.index = 0
		self.num_axes = 2  # will be auto-detected on first read

		# drawing params (will be recalculated each redraw)
		self.row_height = 80
		self.col_w = max(1, self.canvas_w // SAMPLES)

		# start subprocess reader
		self.proc = None
		self.reader_thread = None
		self.running = True
		self.start_script()

		# schedule redraw
		self.after(30, self.redraw)

	def start_script(self):
		try:
			self.proc = subprocess.Popen([sys.executable, SCRIPT], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
		except Exception as e:
			tk.messagebox.showerror('Error', f'Failed to start {SCRIPT}: {e}')
			self.destroy()
			return

		def reader():
			assert self.proc.stdout is not None
			for raw in self.proc.stdout:
				if not self.running:
					break
				line = raw.strip()
				if not line:
					continue
				parts = line.split(',')
				
				# auto-detect number of axes from CSV format
				if self.num_axes is None:
					if len(parts) == 4:
						# single axis: A,B,Index,Angle
						self.num_axes = 1
					elif len(parts) == 7:
						# dual axis: A1,B1,A2,B2,Index,Angle1,Angle2
						self.num_axes = 2
					else:
						continue  # unknown format
				
				try:
					if self.num_axes == 1:
						# A,B,Index,Angle
						a1 = int(parts[0]); b1 = int(parts[1]); idx = int(parts[2])
						angle1 = float(parts[3])
						self.a1_buf.append(a1)
						self.b1_buf.append(b1)
						self.angle1_label.config(text=f'Axis1: {angle1:.3f}°')
					else:
						# A1,B1,A2,B2,Index,Angle1,Angle2
						a1 = int(parts[0]); b1 = int(parts[1]); a2 = int(parts[2]); b2 = int(parts[3]); idx = int(parts[4])
						angle1 = float(parts[5]); angle2 = float(parts[6])
						self.a1_buf.append(a1)
						self.b1_buf.append(b1)
						self.a2_buf.append(a2)
						self.b2_buf.append(b2)
						self.angle1_label.config(text=f'Axis1: {angle1:.3f}°')
						self.angle2_label.config(text=f'Axis2: {angle2:.3f}°')
				except Exception:
					continue
				self.index = idx

		self.reader_thread = threading.Thread(target=reader, daemon=True)
		self.reader_thread.start()

	def redraw(self):
		# update current canvas size (support window resize)
		try:
			self.canvas_w = max(100, self.canvas.winfo_width())
			self.canvas_h = max(100, self.canvas.winfo_height())
		except Exception:
			pass
		self.canvas.delete('all')
		# recalc drawing params based on number of axes
		num_rows = self.num_axes if self.num_axes else 2  # default to 2 for display
		self.row_height = max(40, self.canvas_h // (num_rows + 1))  # +1 for margin
		self.col_w = max(1, self.canvas_w // SAMPLES)
		# draw rows for A1,B1,A2,B2 (or just A1,B1 for single axis)
		padding = 4
		buffers = [(self.a1_buf, 'A1'), (self.b1_buf, 'B1')]
		if self.num_axes == 2:
			buffers.extend([(self.a2_buf, 'A2'), (self.b2_buf, 'B2')])
		
		for i, (buf, label) in enumerate(buffers):
			y0 = padding + i * self.row_height
			y1 = y0 + self.row_height - 8
			# draw row label
			self.canvas.create_text(8, (y0+y1)/2, anchor='w', text=label, font=('Consolas', 12))
			# draw waveform columns from right to left
			x = self.canvas_w - self.col_w
			# make a snapshot copy to avoid deque mutation during iteration
			try:
				snapshot = list(buf)
			except Exception:
				continue
			for v in reversed(snapshot):
				color = 'green' if v else 'lightgrey'
				# represent high as taller bar
				if v:
					self.canvas.create_rectangle(x, y0+2, x+self.col_w-1, y1-2, fill=color, outline='')
				else:
					self.canvas.create_rectangle(x, y1-6, x+self.col_w-1, y1-2, fill=color, outline='')
				x -= self.col_w

		# draw index indicator at top-right
		idx_color = 'red' if self.index else 'lightgrey'
		self.canvas.create_oval(self.canvas_w-36, 6, self.canvas_w-10, 30, fill=idx_color, outline='black')
		self.canvas.create_text(self.canvas_w-60, 18, text='IDX', font=('Consolas', 10))

		if self.running:
			self.after(30, self.redraw)

	def on_close(self):
		self.running = False
		try:
			if self.proc and self.proc.poll() is None:
				self.proc.terminate()
		except Exception:
			pass
		self.destroy()


def main():
	app = LiveDisplay()
	app.protocol('WM_DELETE_WINDOW', app.on_close)
	app.mainloop()


if __name__ == '__main__':
	main()
