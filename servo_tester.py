# servo_controller_full.py
import sys
import json
import time
import math
import queue
import threading
from datetime import datetime
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import serial
import serial.tools.list_ports

# ================= CONFIG =================
DEFAULT_BAUD = 115200
TOTAL_CHANNELS = 16
PW_MIN = 500    # us
PW_MAX = 2500   # us
DEFAULT_SPEED_DPS = 120  # degrees per second (movement rate limit)
COMMAND_INTERVAL_MS = 50  # minimum ms between serial commands per channel
# ==========================================

def now_ts():
    return datetime.now().strftime("[%H:%M:%S] ")

def pulse_from_angle(angle):
    return int(PW_MIN + (angle / 180.0) * (PW_MAX - PW_MIN))

def angle_from_pulse(pw):
    return int(((pw - PW_MIN) / (PW_MAX - PW_MIN)) * 180)

def checksum(ch, angle, pw):
    return (ch + angle + pw) & 0xFF

def build_cmd(ch, angle, pw):
    return f"S,{ch},{angle},{pw},{checksum(ch, angle, pw)}*"

# ============ Serial thread (safe writer + reader) ============
class SerialThread(QThread):
    dataReceived = pyqtSignal(str)
    connectionChanged = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.port = None
        self.baud = DEFAULT_BAUD
        self.ser = None
        self._running = True
        self.write_queue = queue.Queue()
        self._connected = False

    def open(self, port, baud=DEFAULT_BAUD):
        self.port = port
        self.baud = baud
        # If already open, close
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except:
                pass
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self._connected = True
            self.connectionChanged.emit(True)
            self.log(f"Opened {self.port}@{self.baud}")
        except Exception as e:
            self.ser = None
            self._connected = False
            self.connectionChanged.emit(False)
            self.log(f"Open failed: {e}")

    def close(self):
        self._running = False
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
        self._connected = False
        self.connectionChanged.emit(False)

    def run(self):
        while self._running:
            # handle writes
            try:
                while not self.write_queue.empty() and self.ser and self.ser.is_open:
                    msg = self.write_queue.get_nowait()
                    try:
                        self.ser.write(msg.encode())
                        # small pause to avoid flooding USB
                        time.sleep(0.004)
                    except Exception as e:
                        self.log(f"Write error: {e}")
                        break
            except queue.Empty:
                pass

            # handle reads
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode(errors="ignore").strip()
                    if line:
                        self.dataReceived.emit(line)
                except Exception as e:
                    self.log(f"Read error: {e}")

            time.sleep(0.01)  # main loop sleep

    def send(self, text):
        # text must include trailing '*' (protocol)
        if not text.endswith('*'):
            text = text + '*'
        self.write_queue.put(text)

    def log(self, s):
        self.dataReceived.emit("LOG: " + str(s))

# ============== Movement manager ==============
class MovementManager(QObject):
    """
    Handles smoothing, rate limiting and per-channel interpolation.
    Uses a single worker thread to process movement tasks and send commands through the serial thread.
    """
    logSignal = pyqtSignal(str)

    def __init__(self, serial_thread: SerialThread):
        super().__init__()
        self.serial_thread = serial_thread
        self.targets = [None] * TOTAL_CHANNELS  # target angle
        self.current = [90] * TOTAL_CHANNELS  # current known/assumed angle (start at 90°)
        self.speed_dps = DEFAULT_SPEED_DPS  # degrees per second
        self._stop_flag = threading.Event()
        self._lock = threading.Lock()
        self.last_sent_time = [0] * TOTAL_CHANNELS
        self.min_cmd_interval = COMMAND_INTERVAL_MS / 1000.0
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

    def set_speed(self, dps):
        with self._lock:
            self.speed_dps = max(1, float(dps))
        self._log(f"Speed limit set to {self.speed_dps}°/s")

    def move_to(self, ch, target_angle):
        with self._lock:
            self.targets[ch] = float(max(0, min(180, target_angle)))
        self._log(f"Queued move CH{ch} -> {self.targets[ch]}°")

    def stop_all(self):
        with self._lock:
            for i in range(TOTAL_CHANNELS):
                self.targets[i] = None
        self._stop_flag.set()
        self._log("STOP ALL issued")

    def sweep(self, ch_list, speed_dps=None, dwell=0.25):
        """Queue a sweep: move each channel in ch_list from min to max and back once."""
        def sweep_thread():
            sp = speed_dps if speed_dps is not None else self.speed_dps
            for ch in ch_list:
                if self._stop_flag.is_set():
                    break
                # read calibration min/max from GUI? We'll use defaults 0-180 here.
                min_a, max_a = 0, 180
                # sweep up
                self.move_to(ch, max_a)
                # wait until reach
                while True:
                    time.sleep(0.05)
                    with self._lock:
                        if self.targets[ch] is None:
                            break
                        cur = self.current[ch]
                        if abs(cur - max_a) < 1:
                            break
                    if self._stop_flag.is_set():
                        break
                time.sleep(dwell)
                # sweep down
                self.move_to(ch, min_a)
                while True:
                    time.sleep(0.05)
                    with self._lock:
                        cur = self.current[ch]
                        if abs(cur - min_a) < 1:
                            break
                    if self._stop_flag.is_set():
                        break
                time.sleep(dwell)
            self._log("Sweep finished")
        threading.Thread(target=sweep_thread, daemon=True).start()

    def _worker_loop(self):
        while True:
            if self._stop_flag.is_set():
                # clear flag and continue (this prevents new moves until user clears)
                self._stop_flag.clear()

            now = time.time()
            with self._lock:
                # create local copies to avoid holding lock during sends
                targets_copy = list(self.targets)
                speed = self.speed_dps

            for ch, targ in enumerate(targets_copy):
                if targ is None:
                    continue
                cur = self.current[ch]
                if abs(targ - cur) < 0.5:
                    # reached target
                    with self._lock:
                        self.current[ch] = targ
                        self.targets[ch] = None
                    continue

                # compute step based on speed and elapsed time
                dt = 0.05  # worker tick interval
                max_delta = speed * dt  # degrees per tick
                delta = targ - cur
                step = max(-max_delta, min(max_delta, delta))
                new_angle = cur + step

                # rate limit per-channel commands
                if now - self.last_sent_time[ch] >= self.min_cmd_interval:
                    pw = pulse_from_angle(int(round(new_angle)))
                    cmd = build_cmd(ch, int(round(new_angle)), pw)
                    self.serial_thread.send(cmd)
                    self.last_sent_time[ch] = now
                    # update current angle estimate only after sending
                    with self._lock:
                        self.current[ch] = new_angle

            time.sleep(0.05)

    def _log(self, m):
        self.logSignal.emit(now_ts() + m)

# ================= Main GUI =================
class ServoControllerGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Advanced 16-Channel Servo Controller")
        self.resize(1100, 720)

        # Serial thread
        self.serial_thread = SerialThread()
        self.serial_thread.dataReceived.connect(self.on_serial_data)
        self.serial_thread.connectionChanged.connect(self.on_connection_changed)
        self.serial_thread.start()

        # Movement manager
        self.movement_mgr = MovementManager(self.serial_thread)
        self.movement_mgr.logSignal.connect(self.log)

        # UI state
        self.channels = []
        self.angle_edits = []
        self.pw_edits = []
        self.min_spin = []
        self.max_spin = []

        self._build_ui()
        self.load_calibration()  # try load existing calibration

        self.log("App started")

    def _build_ui(self):
        layout = QGridLayout(self)
        # Left pane: logs & controls
        left_v = QVBoxLayout()
        layout.addLayout(left_v, 0, 0, 6, 1)

        # Port selection
        port_h = QHBoxLayout()
        self.port_combo = QComboBox()
        self.refresh_ports()
        btn_refresh = QPushButton("Refresh")
        btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connect)

        port_h.addWidget(QLabel("COM Port:"))
        port_h.addWidget(self.port_combo)
        port_h.addWidget(btn_refresh)
        port_h.addWidget(self.btn_connect)
        left_v.addLayout(port_h)

        # Speed control
        speed_h = QHBoxLayout()
        speed_h.addWidget(QLabel("Max speed (°/s):"))
        self.speed_spin = QSpinBox(); self.speed_spin.setRange(1, 1000); self.speed_spin.setValue(DEFAULT_SPEED_DPS)
        self.speed_spin.valueChanged.connect(lambda v: self.movement_mgr.set_speed(v))
        speed_h.addWidget(self.speed_spin)
        left_v.addLayout(speed_h)

        # Buttons
        btns_h = QHBoxLayout()
        self.btn_all90 = QPushButton("All to 90°"); self.btn_all90.clicked.connect(self.set_all_90)
        self.btn_stop = QPushButton("STOP ALL"); self.btn_stop.clicked.connect(self.stop_all)
        self.btn_save = QPushButton("Save Config"); self.btn_save.clicked.connect(self.save_config)
        self.btn_load = QPushButton("Load Config"); self.btn_load.clicked.connect(self.load_config)
        self.btn_sweep = QPushButton("Sweep Test"); self.btn_sweep.clicked.connect(self.sweep_test)

        btns_h.addWidget(self.btn_all90); btns_h.addWidget(self.btn_stop)
        btns_h.addWidget(self.btn_save); btns_h.addWidget(self.btn_load); btns_h.addWidget(self.btn_sweep)
        left_v.addLayout(btns_h)

        # Log box
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        left_v.addWidget(QLabel("Logs:"))
        left_v.addWidget(self.log_box)

        # Right pane: channel controls grid
        grid = QGridLayout()
        layout.addLayout(grid, 0, 1, 6, 3)

        # header
        grid.addWidget(QLabel("CH"), 0, 0)
        grid.addWidget(QLabel("Slider"), 0, 1)
        grid.addWidget(QLabel("Angle °"), 0, 2)
        grid.addWidget(QLabel("Pulse µs"), 0, 3)
        grid.addWidget(QLabel("Min"), 0, 4)
        grid.addWidget(QLabel("Max"), 0, 5)
        grid.addWidget(QLabel("Send Now"), 0, 6)

        for ch in range(TOTAL_CHANNELS):
            lbl = QLabel(str(ch))
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 180)
            slider.setValue(90)
            slider.valueChanged.connect(lambda v, c=ch: self.on_slider_change(c, v))

            angle_edit = QLineEdit("90"); angle_edit.setFixedWidth(50); angle_edit.setReadOnly(True)
            pw_edit = QLineEdit(str(pulse_from_angle(90))); pw_edit.setFixedWidth(70)
            pw_edit.editingFinished.connect(lambda c=ch, e=pw_edit: self.on_pw_edit(c, e))

            min_spin = QSpinBox(); min_spin.setRange(0, 180); min_spin.setValue(0)
            max_spin = QSpinBox(); max_spin.setRange(0, 180); max_spin.setValue(180)

            btn_send = QPushButton("Send")
            btn_send.clicked.connect(lambda _, c=ch: self.send_now(c))

            row = ch + 1
            grid.addWidget(lbl, row, 0)
            grid.addWidget(slider, row, 1)
            grid.addWidget(angle_edit, row, 2)
            grid.addWidget(pw_edit, row, 3)
            grid.addWidget(min_spin, row, 4)
            grid.addWidget(max_spin, row, 5)
            grid.addWidget(btn_send, row, 6)

            self.channels.append(slider)
            self.angle_edits.append(angle_edit)
            self.pw_edits.append(pw_edit)
            self.min_spin.append(min_spin)
            self.max_spin.append(max_spin)

        # bottom: calibration filename display
        self.cal_label = QLabel("Calibration: (not saved)")
        layout.addWidget(self.cal_label, 6, 0, 1, 4)

    # ---------------- UI helpers ----------------
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(f"{p.device} - {p.description}", p.device)
        if self.port_combo.count() == 0:
            self.port_combo.addItem("No ports", "")

    def toggle_connect(self):
        if self.serial_thread.ser and self.serial_thread.ser.is_open:
            # disconnect
            try:
                self.serial_thread.ser.close()
            except:
                pass
            self.btn_connect.setText("Connect")
            self.log("Disconnected")
        else:
            port = self.port_combo.currentData()
            if not port:
                port = self.port_combo.currentText().split(' - ')[0]
            self.serial_thread.open(port, DEFAULT_BAUD)
            # update button text on success handled by signal

    def on_connection_changed(self, connected):
        self.btn_connect.setText("Disconnect" if connected else "Connect")
        self.log("Connected" if connected else "Not connected")

    def log(self, text):
        self.log_box.append(now_ts() + str(text))

    # ---------------- Channel interactions ----------------
    def on_slider_change(self, ch, val):
        # update UI fields and queue movement
        self.angle_edits[ch].setText(str(int(val)))
        pw = pulse_from_angle(int(val))
        self.pw_edits[ch].setText(str(pw))
        # Use movement manager to smoothly move with rate limit
        self.movement_mgr.move_to(ch, val)

    def on_pw_edit(self, ch, edit_widget):
        try:
            pw = int(edit_widget.text())
        except:
            return
        pw = max(PW_MIN, min(PW_MAX, pw))
        ang = angle_from_pulse(pw)
        self.channels[ch].setValue(ang)
        # movement manager will pick up slider change and move.

    def send_now(self, ch):
        ang = int(self.angle_edits[ch].text())
        pw = pulse_from_angle(ang)
        cmd = build_cmd(ch, ang, pw)
        self.serial_thread.send(cmd)
        self.log(f"Sent NOW CH{ch}: {ang}°, {pw}µs")

    # ---------------- Buttons ----------------
    def set_all_90(self):
        for ch in range(TOTAL_CHANNELS):
            self.channels[ch].setValue(90)
        self.log("All channels set to 90° (queued)")

    def stop_all(self):
        # cancel all movements
        self.movement_mgr.stop_all()
        # send immediate commands to hold current positions (prevents drift)
        for ch in range(TOTAL_CHANNELS):
            ang = int(self.angle_edits[ch].text())
            pw = pulse_from_angle(ang)
            cmd = build_cmd(ch, ang, pw)
            self.serial_thread.send(cmd)
        self.log("STOP ALL: movements cancelled and current positions sent")

    def sweep_test(self):
        # Sweep all channels sequentially once
        ch_list = list(range(TOTAL_CHANNELS))
        self.movement_mgr.sweep(ch_list, speed_dps=self.speed_spin.value())
        self.log("Started sweep test")

    # ---------------- Config save/load ----------------
    def save_config(self):
        config = {
            "angles": [int(edit.text()) for edit in self.angle_edits],
            "calibration": [{"min": s.value(), "max": m.value()} for s, m in zip(self.min_spin, self.max_spin)],
            "speed_dps": self.speed_spin.value()
        }
        fname, _ = QFileDialog.getSaveFileName(self, "Save Config", "", "JSON Files (*.json)")
        if not fname:
            return
        try:
            with open(fname, "w") as f:
                json.dump(config, f, indent=2)
            self.cal_label.setText(f"Calibration: {fname}")
            self.log(f"Config saved to {fname}")
        except Exception as e:
            self.log(f"Save failed: {e}")

    def load_config(self):
        fname, _ = QFileDialog.getOpenFileName(self, "Load Config", "", "JSON Files (*.json)")
        if not fname:
            return
        try:
            with open(fname, "r") as f:
                config = json.load(f)
            angles = config.get("angles", [])
            for ch, ang in enumerate(angles):
                if ch < TOTAL_CHANNELS:
                    self.channels[ch].setValue(int(ang))
            calib = config.get("calibration", [])
            for ch in range(min(len(calib), TOTAL_CHANNELS)):
                self.min_spin[ch].setValue(calib[ch].get("min", 0))
                self.max_spin[ch].setValue(calib[ch].get("max", 180))
            sp = config.get("speed_dps")
            if sp:
                self.speed_spin.setValue(int(sp))
                self.movement_mgr.set_speed(int(sp))
            self.cal_label.setText(f"Calibration: {fname}")
            self.log(f"Config loaded from {fname}")
        except Exception as e:
            self.log(f"Load failed: {e}")

    def load_calibration(self):
        # try default filename
        try:
            with open("servo_calibration.json", "r") as f:
                config = json.load(f)
            angles = config.get("angles", [])
            for ch, ang in enumerate(angles):
                if ch < TOTAL_CHANNELS:
                    self.channels[ch].setValue(int(ang))
            calib = config.get("calibration", [])
            for ch in range(min(len(calib), TOTAL_CHANNELS)):
                self.min_spin[ch].setValue(calib[ch].get("min", 0))
                self.max_spin[ch].setValue(calib[ch].get("max", 180))
            sp = config.get("speed_dps")
            if sp:
                self.speed_spin.setValue(int(sp))
                self.movement_mgr.set_speed(int(sp))
            self.cal_label.setText("Calibration: servo_calibration.json (auto)")
            self.log("Auto-loaded servo_calibration.json")
        except:
            pass  # no auto-load if missing

    # ---------------- Serial RX ----------------
    def on_serial_data(self, line):
        # Display line in log. Could parse feedback messages here to update UI.
        self.log("RX " + line)

# ===== main =====
def main():
    app = QApplication(sys.argv)
    gui = ServoControllerGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
