"""

Copyright (c) 2019 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

"""

import PyQt5
from PyQt5 import Qt, QtCore, QtGui, QtWidgets

import math
import time

from functools import partial

import packet


class DIOChannel(object):
    def __init__(self):
        self.name = "dio"
        self.label = "DIO channel"
        self.devid = 0x00
        self.bank = 0
        self.channel = 0
        self.invert = False
        self.enable_label = "Enable"
        self.disable_label = "Disable"
        self.enabled_label = "On"
        self.disabled_label = "Off"
        self.raw_value = None
        self.value = None
        self.default_value = None
        self.safe_value = None
        self.timeout = 1.0
        self.timeout_limit = 0
        self.controls = []
        self.send_pkt = []

    def convert_raw_value(self, value):
        if value is None:
            return None
        return (not value) if self.invert else value

    def set_raw_value(self, value):
        self.raw_value = value

        if value is not None and self.timeout > 0:
            self.timeout_limit = time.time() + self.timeout

        for c in self.controls:
            c.set_raw_value(value)

        self.set_value(self.convert_raw_value(value))

    def set_value(self, value):
        self.value = value

        for c in self.controls:
            c.set_value(value)

    def command_state(self, value):
        pkt = packet.DIOSetBitPacket()
        pkt.dest = self.devid
        pkt.flags = 0
        pkt.bank = self.bank
        pkt.bit = self.channel
        pkt.state = (not value) if self.invert else value
        self.send_pkt(pkt)

    def command_default_state(self):
        if self.default_value is not None:
            self.command_state(self.default_value)

    def command_safe_state(self):
        if self.safe_value is not None:
            self.command_state(self.safe_value)

    def tick(self):
        if self.timeout_limit and self.timeout_limit < time.time():
            self.set_raw_value(None)


class AnalogChannel(object):
    def __init__(self):
        self.name = "analog"
        self.label = "Analog channel"
        self.devid = 0x00
        self.bank = 0
        self.channel = 0
        self.zero = 0.0
        self.slope = 1.0
        self.unit = "V"
        self.format = "{:6.2f} {}"
        self.unknown_format = "---.-- {}"
        self.raw_value = None
        self.value = None
        self.timeout = 1.0
        self.timeout_limit = 0
        self.controls = []

    def convert_raw_value(self, value):
        if value is None:
            return None
        return (value - self.zero) * self.slope

    def set_raw_value(self, value):
        self.raw_value = value

        if value is not None and self.timeout > 0:
            self.timeout_limit = time.time() + self.timeout

        for c in self.controls:
            c.set_raw_value(value)

        self.set_value(self.convert_raw_value(value))

    def set_value(self, value):
        self.value = value

        for c in self.controls:
            c.set_value(value)

    def tick(self):
        if self.timeout_limit and self.timeout_limit < time.time():
            self.set_raw_value(None)


class DIOInputControl(QtWidgets.QGroupBox):
    def __init__(self, channel):
        self.channel = channel
        self.channel.controls.append(self)
        self.name = self.channel.label

        super(DIOInputControl, self).__init__()

        self.setTitle(QtWidgets.QApplication.translate("MainWindow", self.name, None))

        self.vbox1 = QtWidgets.QVBoxLayout(self)

        self.statusLabel = QtWidgets.QLabel("Unknown")
        self.statusLabel.setStyleSheet('background-color: silver')
        self.statusLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.statusLabel.setContentsMargins(20, 20, 20, 20)
        self.vbox1.addWidget(self.statusLabel, 0)

        self.detailsLabel = QtWidgets.QLabel("0x00:0.0  Raw -")
        self.vbox1.addWidget(self.detailsLabel)

        self.set_value(None)
        self.set_raw_value(None)

    def set_name(self, name):
        self.name = name
        self.setTitle(name)

    def set_value(self, val):
        if val is None:
            self.statusLabel.setText("Unknown")
            self.statusLabel.setStyleSheet('background-color: silver')
        elif val:
            self.statusLabel.setText("On")
            self.statusLabel.setStyleSheet('background-color: green; color: white')
        else:
            self.statusLabel.setText("Off")
            self.statusLabel.setStyleSheet('background-color: red; color: white')

    def set_raw_value(self, val):
        if val is None:
            s = "-"
        else:
            s = "{:d}".format(bool(val))
        if self.channel.invert:
            s += " (Inv)"
        self.detailsLabel.setText("{:#04x}:{:d}.{:d}  Raw: {}".format(self.channel.devid, self.channel.bank, self.channel.channel, s))


class DIOOutputControl(QtWidgets.QGroupBox):
    def __init__(self, channel):
        self.channel = channel
        self.channel.controls.append(self)
        self.name = self.channel.label

        super(DIOOutputControl, self).__init__()

        self.setTitle(QtWidgets.QApplication.translate("MainWindow", self.name, None))

        self.vbox1 = QtWidgets.QVBoxLayout(self)

        self.hbox1 = QtWidgets.QHBoxLayout()
        self.vbox1.addLayout(self.hbox1)

        self.vbox2 = QtWidgets.QVBoxLayout()
        self.hbox1.addLayout(self.vbox2)

        self.enableButton = QtWidgets.QPushButton(QtWidgets.QApplication.translate("MainWindow", self.channel.enable_label, None), self)
        self.enableButton.clicked.connect(self.on_enable_clicked)
        self.vbox2.addWidget(self.enableButton)

        self.disableButton = QtWidgets.QPushButton(QtWidgets.QApplication.translate("MainWindow", self.channel.disable_label, None), self)
        self.disableButton.clicked.connect(self.on_disable_clicked)
        self.vbox2.addWidget(self.disableButton)

        self.statusLabel = QtWidgets.QLabel("Unknown")
        self.statusLabel.setStyleSheet('background-color: silver')
        self.statusLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.statusLabel.setContentsMargins(20, 20, 20, 20)
        self.hbox1.addWidget(self.statusLabel, 0)

        self.detailsLabel = QtWidgets.QLabel("0x00:0.0  Raw -")
        self.vbox1.addWidget(self.detailsLabel)

        self.set_value(None)
        self.set_raw_value(None)

    def set_name(self, name):
        self.name = name
        self.setTitle(name)

    def set_value(self, val):
        if val is None:
            self.statusLabel.setText("Unknown")
            self.statusLabel.setStyleSheet('background-color: silver')
        elif val:
            self.statusLabel.setText(self.channel.enabled_label)
            self.statusLabel.setStyleSheet('background-color: green; color: white')
        else:
            self.statusLabel.setText(self.channel.disabled_label)
            self.statusLabel.setStyleSheet('background-color: red; color: white')

    def set_raw_value(self, val):
        if val is None:
            s = "-"
        else:
            s = "{:d}".format(bool(val))
        if self.channel.invert:
            s += " (Inv)"
        self.detailsLabel.setText("{:#04x}:{:d}.{:d}  Raw: {}".format(self.channel.devid, self.channel.bank, self.channel.channel, s))

    def on_enable_clicked(self):
        self.channel.command_state(1)

    def on_disable_clicked(self):
        self.channel.command_state(0)


class AnalogControl(QtWidgets.QGroupBox):
    def __init__(self, channel):
        self.channel = channel
        self.channel.controls.append(self)
        self.name = self.channel.label

        super(AnalogControl, self).__init__()

        self.setTitle(QtWidgets.QApplication.translate("MainWindow", self.name, None))

        self.vbox1 = QtWidgets.QVBoxLayout(self)

        self.statusLabel = QtWidgets.QLabel(self.channel.unknown_format.format(self.channel.unit))
        font = QtGui.QFont("monospace")
        font.setPointSize(20)
        font.setStyleHint(QtGui.QFont.TypeWriter)
        self.statusLabel.setFont(font)
        self.statusLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.statusLabel.setContentsMargins(10, 5, 10, 5)
        self.vbox1.addWidget(self.statusLabel, 0)

        self.detailsLabel = QtWidgets.QLabel("0x00:0.0  Raw: -.---- V")
        self.vbox1.addWidget(self.detailsLabel)

        self.set_value(None)
        self.set_raw_value(None)

    def set_name(self, name):
        self.name = name
        self.setTitle(name)

    def set_value(self, val):
        try:
            self.statusLabel.setText(self.channel.format.format(val, self.channel.unit))
            self.statusLabel.setStyleSheet('')
        except Exception:
            self.statusLabel.setText(self.channel.unknown_format.format(self.channel.unit))
            self.statusLabel.setStyleSheet('background-color: silver')

    def set_raw_value(self, val):
        if val is None:
            s = "-.----"
        else:
            s = "{:1.4f}".format(val)
        self.detailsLabel.setText("{:#04x}:{:d}.{:d}  Raw: {} V".format(self.channel.devid, self.channel.bank, self.channel.channel, s))


class CommandControl(QtWidgets.QGroupBox):
    def __init__(self):
        self.name = "Command"

        super(CommandControl, self).__init__()

        self.setTitle(QtWidgets.QApplication.translate("MainWindow", "Command", None))

        self.vbox1 = QtWidgets.QVBoxLayout(self)

        self.sendButton = QtWidgets.QPushButton(QtWidgets.QApplication.translate("MainWindow", "Send", None), self)
        self.vbox1.addWidget(self.sendButton)

        self.detailsLabel = QtWidgets.QLabel("Dev 0x00  Cmd 0x00000000")
        self.vbox1.addWidget(self.detailsLabel)

    def set_name(self, name):
        self.name = name
        self.setTitle(name)


class FlightComputerStatusControl(QtWidgets.QGroupBox):
    def __init__(self, gps=False):
        self.name = "Flight Computer Status"
        self.devid = 0

        super(FlightComputerStatusControl, self).__init__()

        self.setTitle(QtWidgets.QApplication.translate("MainWindow", "Flight Computer Status", None))

        self.vbox1 = QtWidgets.QVBoxLayout(self)

        self.form = QtWidgets.QFormLayout()
        self.vbox1.addLayout(self.form)

        self.flight_phase_label = QtWidgets.QLabel("-")
        self.form.addRow("Flight phase:", self.flight_phase_label)
        self.arm_status_label = QtWidgets.QLabel("-")
        self.form.addRow("Arm status:", self.arm_status_label)
        self.em_status_label = QtWidgets.QLabel("- / -")
        self.form.addRow("E-match status:", self.em_status_label)
        self.log_status_label = QtWidgets.QLabel("-")
        self.form.addRow("Log status:", self.log_status_label)

        self.baro_pressure_label = QtWidgets.QLabel("---- Pa")
        self.form.addRow("Baro pressure:", self.baro_pressure_label)
        self.baro_temperature_label = QtWidgets.QLabel("--.-- \N{DEGREE SIGN}C")
        self.form.addRow("Baro temperature:", self.baro_temperature_label)
        self.imu_accel_label = QtWidgets.QLabel("--.-- m/s\N{SUPERSCRIPT TWO}")
        self.form.addRow("IMU accelerometer:", self.imu_accel_label)

        self.baro_altitude_label = QtWidgets.QLabel("---- m")
        self.form.addRow("Baro altitude:", self.baro_altitude_label)
        self.baro_pad_altitude_label = QtWidgets.QLabel("---- m")
        self.form.addRow("Baro pad_altitude:", self.baro_pad_altitude_label)
        self.baro_speed_label = QtWidgets.QLabel("---- m/s")
        self.form.addRow("Baro speed:", self.baro_speed_label)
        self.imu_speed_label = QtWidgets.QLabel("--.-- m/s")
        self.form.addRow("IMU speed:", self.imu_speed_label)
        self.imu_altitude_label = QtWidgets.QLabel("--.-- m/s")
        self.form.addRow("IMU altitude:", self.imu_altitude_label)

        self.gps_lat_label = QtWidgets.QLabel("--.-------\N{DEGREE SIGN} N")
        self.gps_lon_label = QtWidgets.QLabel("---.-------\N{DEGREE SIGN} E")
        self.gps_alt_label = QtWidgets.QLabel("---.-- m")
        if gps:
            self.form.addRow("GPS Latitude:", self.gps_lat_label)
            self.form.addRow("GPS Longitude:", self.gps_lon_label)
            self.form.addRow("GPS Altitude:", self.gps_alt_label)

    def set_name(self, name):
        self.name = name
        self.setTitle(name)

    def handle_packet(self, pkt):
        if pkt.source == self.devid:
            if isinstance(pkt, packet.DIOStatePacket):
                pass
            elif isinstance(pkt, packet.GpsPositionPacket):
                self.gps_lat_label.setText("{:010.7f}\N{DEGREE SIGN} {}".format(abs(pkt.latitude), 'S' if pkt.latitude < 0 else 'N'))
                self.gps_lon_label.setText("{:011.7f}\N{DEGREE SIGN} {}".format(abs(pkt.longitude), 'W' if pkt.longitude < 0 else 'E'))
                self.gps_alt_label.setText("{:0.2f} m".format(pkt.altitude))
            elif isinstance(pkt, packet.FlightStatusPacket):
                #self.time
                if pkt.flight_phase == 0:
                    self.flight_phase_label.setText("PAD")
                elif pkt.flight_phase == 1:
                    self.flight_phase_label.setText("ASCENT 1")
                elif pkt.flight_phase == 2:
                    self.flight_phase_label.setText("ASCENT 2")
                elif pkt.flight_phase == 3:
                    self.flight_phase_label.setText("ASCENT 3")
                elif pkt.flight_phase == 4:
                    self.flight_phase_label.setText("DESCENT 1")
                elif pkt.flight_phase == 5:
                    self.flight_phase_label.setText("DESCENT 2")
                else:
                    self.flight_phase_label.setText("UNKNOWN")
                self.arm_status_label.setText("Armed" if pkt.status_flags & (1 << 0) else "Disarmed")
                self.log_status_label.setText("Good" if pkt.status_flags & (1 << 1) else "Failed")
                self.em_status_label.setText(("Good" if pkt.status_flags & (1 << 4) else "Bad") + " / " + ("Good" if pkt.status_flags & (1 << 5) else "Bad"))
                self.baro_pressure_label.setText("{} Pa".format(pkt.baro_pressure))
                self.baro_temperature_label.setText("{:0.2f} \N{DEGREE SIGN}C".format(pkt.baro_temperature))
                self.imu_accel_label.setText("{:0.2f} m/s\N{SUPERSCRIPT TWO}".format(pkt.imu_accel))
                self.baro_altitude_label.setText("{:0.2f} m".format(pkt.baro_altitude))
                self.baro_pad_altitude_label.setText("{:0.2f} m".format(pkt.baro_pad_altitude))
                self.baro_speed_label.setText("{:0.2f} m/s".format(pkt.baro_speed))
                self.imu_speed_label.setText("{:0.2f} m/s".format(pkt.imu_speed))
                self.imu_altitude_label.setText("{:0.2f} m".format(pkt.imu_altitude))


class LaunchSequenceControl(QtWidgets.QGroupBox):
    def __init__(self, parent):
        self.name = "Launch Sequence"
        self.devid = 0

        self.dio_channels = parent.dio_channels
        self.send_pkt = parent.send_pkt

        self.key_status = False
        self.arm_status = False
        self.t0_time = 0
        self.t0 = -10
        self.t = self.t0
        self.em_fired = False
        self.mvasa_opened = False

        super(LaunchSequenceControl, self).__init__()

        self.setTitle(QtWidgets.QApplication.translate("MainWindow", "Launch Sequence", None))

        self.vbox1 = QtWidgets.QVBoxLayout(self)

        self.form = QtWidgets.QFormLayout()
        self.vbox1.addLayout(self.form)

        self.arming_status_label = QtWidgets.QLabel("-")
        self.form.addRow("Arming status:", self.arming_status_label)
        self.em_status_label = QtWidgets.QLabel("- / -")
        self.form.addRow("E-match status:", self.em_status_label)
        self.sequence_status_label = QtWidgets.QLabel("DISABLED")
        self.form.addRow("Sequence status:", self.sequence_status_label)
        self.sequence_time_label = QtWidgets.QLabel("T-00:10")
        self.form.addRow("Sequence time:", self.sequence_time_label)

        self.start_button = QtWidgets.QPushButton(QtWidgets.QApplication.translate("MainWindow", "Start sequence", None), self)
        self.start_button.setEnabled(False)
        self.start_button.clicked.connect(self.on_start_clicked)
        self.vbox1.addWidget(self.start_button)

    def set_name(self, name):
        self.name = name
        self.setTitle(name)

    def handle_packet(self, pkt):
        if pkt.source == self.devid:
            if isinstance(pkt, packet.DIOStatePacket):
                if pkt.bank == 1:
                    self.key_status = pkt.state & (1 << 4)
                    self.arm_status = pkt.state & (1 << 8)
                    self.em_status_label.setText(("Good" if pkt.state & (1 << 0) else "Bad") + " / " + ("Good" if pkt.state & (1 << 1) else "Bad"))

    def on_start_clicked(self):
        self.t0_time = time.time() - self.t0

    def tick(self):
        if self.key_status and self.arm_status:
            self.arming_status_label.setText("ARMED")

            if self.t0_time > 0:
                self.sequence_status_label.setText("RUNNING")
                self.start_button.setEnabled(False)

                self.t = time.time() - self.t0_time

                if self.t >= -5:
                    # fire E-match
                    self.sequence_status_label.setText("EMATCH")
                    if not self.em_fired:
                        self.em_fired = True
                        pkt = packet.CommandPacket()
                        pkt.dest = self.devid
                        pkt.flags = 0
                        pkt.cmd = 0x004001f0
                        pkt.data = b'\x00'
                        self.send_pkt(pkt)

                if self.t >= 0:
                    # open MVASA
                    self.sequence_status_label.setText("MVASA")
                    if not self.mvasa_opened:
                        self.mvasa_opened = True
                        self.dio_channels['v_gse_mvasa'].command_state(True)
            else:
                self.sequence_status_label.setText("READY")
                self.start_button.setEnabled(True)

        else:
            if self.key_status:
                self.arming_status_label.setText("DISARMED")
            else:
                self.arming_status_label.setText("LOCKOUT")
            self.sequence_status_label.setText("DISABLED")
            self.start_button.setEnabled(False)
            self.t0_time = 0
            self.em_fired = False
            self.mvasa_opened = False

        s = math.floor(self.t)
        if s < 0:
            self.sequence_time_label.setText("T-{:02d}:{:02d}".format(int((-s)/60), int((-s)%60)))
        else:
            self.sequence_time_label.setText("T+{:02d}:{:02d}".format(int(s/60), int(s%60)))


class DIODiagnostics(QtWidgets.QDialog):
    
    set_dio = QtCore.pyqtSignal(int, int, int, int)

    def __init__(self, parent=None):
        super(DIODiagnostics, self).__init__(parent)

        self.setObjectName("DIODiagnostics")
        self.setWindowTitle(QtWidgets.QApplication.translate("DIODiagnostics", "DIO Diagnostics", None))

        self.vbox1 = QtWidgets.QVBoxLayout(self)

        self.hbox1 = QtWidgets.QHBoxLayout()
        self.vbox1.addLayout(self.hbox1)

        self.gboxDIOs = {}
        self.gls = {}

        self.labels = {}
        self.setbuttons = {}
        self.resetbuttons = {}
        self.statelabels = {}

        for i in (0x08, 0x40):
            self.labels[i] = []
            self.setbuttons[i] = []
            self.resetbuttons[i] = []
            self.statelabels[i] = []

            self.gboxDIOs[i] = QtWidgets.QGroupBox(QtWidgets.QApplication.translate("DIODiagnostics", "DIOs (0x%x)" % i, None))
            self.hbox1.addWidget(self.gboxDIOs[i])

            self.gls[i] = QtWidgets.QGridLayout(self.gboxDIOs[i])

            for k in range(16):
                l = QtWidgets.QLabel("Ch {}".format(k))
                self.gls[i].addWidget(l, k, 0)
                self.labels[i].append(l)

                sb = QtWidgets.QPushButton("Set")
                sb.clicked.connect(partial(self.do_set, i, 0, k))
                self.gls[i].addWidget(sb, k, 1)
                self.setbuttons[i].append(sb)

                rb = QtWidgets.QPushButton("Reset")
                rb.clicked.connect(partial(self.do_reset, i, 0, k))
                self.gls[i].addWidget(rb, k, 2)
                self.setbuttons[i].append(rb)

                sl = QtWidgets.QLabel("?")
                self.gls[i].addWidget(sl, k, 3)
                self.statelabels[i].append(sl)

        self.buttonBox = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Close, QtCore.Qt.Horizontal, self)
        self.buttonBox.rejected.connect(self.reject)
        self.vbox1.addWidget(self.buttonBox)

    def set_ch_state(self, devid, bank, ch, state):
        if bank != 0:
            return
        if devid not in self.statelabels:
            return
        if ch >= len(self.statelabels[devid]):
            return
        if state is None:
            self.statelabels[devid][ch].setText("?")
            self.statelabels[devid][ch].setStyleSheet('')
        elif state:
            self.statelabels[devid][ch].setText("on")
            self.statelabels[devid][ch].setStyleSheet('background-color: green; color: white')
        else:
            self.statelabels[devid][ch].setText("off")
            self.statelabels[devid][ch].setStyleSheet('background-color: red; color: white')

    def on_update_dio(self, devid, bank, ch, state):
        self.set_ch_state(devid, bank, ch, state)

    def do_set(self, devid, bank, ch):
        self.set_dio.emit(devid, bank, ch, 1)

    def do_reset(self, devid, bank, ch):
        self.set_dio.emit(devid, bank, ch, 0)


class PTDiagnostics(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super(PTDiagnostics, self).__init__(parent)

        self.setObjectName("PTDiagnostics")
        self.setWindowTitle(QtWidgets.QApplication.translate("PTDiagnostics", "PT Diagnostics", None))

        self.vbox1 = QtWidgets.QVBoxLayout(self)

        self.hbox1 = QtWidgets.QHBoxLayout()
        self.vbox1.addLayout(self.hbox1)

        self.gboxPTs = {}
        self.gls = {}

        self.labels = {}
        self.rawlabels = {}

        for i in (0x08, 0x40):
            self.labels[i] = []
            self.rawlabels[i] = []

            self.gboxPTs[i] = QtWidgets.QGroupBox(QtWidgets.QApplication.translate("PTDiagnostics", "PTs (0x%x)" % i, None))
            self.hbox1.addWidget(self.gboxPTs[i])

            self.gls[i] = QtWidgets.QGridLayout(self.gboxPTs[i])

            for k in range(8):
                l = QtWidgets.QLabel("Ch {}".format(k))
                self.gls[i].addWidget(l, k, 0)
                self.labels[i].append(l)

                rl = QtWidgets.QLabel("-.---- v")
                self.gls[i].addWidget(rl, k, 1)
                self.rawlabels[i].append(rl)

        self.buttonBox = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Close, QtCore.Qt.Horizontal, self)
        self.buttonBox.rejected.connect(self.reject)
        self.vbox1.addWidget(self.buttonBox)

    def set_ch_value(self, devid, ch, value):
        if devid not in self.rawlabels:
            return
        if ch >= len(self.rawlabels[devid]):
            return
        if value is None:
            self.rawlabels[devid][ch].setText("-.---- v")
        else:
            self.rawlabels[devid][ch].setText("{:.4f} v".format(value))

    def on_update_pt(self, devid, ch, value):
        self.set_ch_value(devid, ch, value)


class ConnectDialogSerial(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super(ConnectDialogSerial, self).__init__(parent)

        self.setObjectName("ConnectDialogSerial")
        self.setWindowTitle(QtWidgets.QApplication.translate("MainWindow", "Connect (Serial)", None))

        self.vbox1 = QtWidgets.QVBoxLayout(self)

        self.gboxSerialPort = QtWidgets.QGroupBox(QtWidgets.QApplication.translate("MainWindow", "Serial Port", None))
        self.vbox1.addWidget(self.gboxSerialPort)

        self.hbox1 = QtWidgets.QHBoxLayout(self.gboxSerialPort)

        self.form1 = QtWidgets.QFormLayout()
        self.hbox1.addLayout(self.form1)

        self.portCombo = QtWidgets.QComboBox()
        self.form1.addRow("Port", self.portCombo)

        self.speedCombo = QtWidgets.QComboBox()
        self.speedCombo.addItems(["4000000", "3000000", "2500000", "2000000", "1500000", "1152000", "1000000", "921600",
            "576000", "500000", "460800", "230400", "115200", "38400", "19200", "9600", "4800", "2400", "1200", "600", "300"])
        self.speedCombo.setCurrentIndex(12)
        self.form1.addRow("Speed", self.speedCombo)

        self.flowCombo = QtWidgets.QComboBox()
        self.flowCombo.addItems(["none", "hardware (RTS/CTS)", "software (XON/XOFF)"])
        self.form1.addRow("Flow control", self.flowCombo)

        self.form2 = QtWidgets.QFormLayout()
        self.hbox1.addLayout(self.form2)

        self.parityCombo = QtWidgets.QComboBox()
        self.parityCombo.addItems(["none", "even", "odd"])
        self.form2.addRow("Parity", self.parityCombo)

        self.bitsCombo = QtWidgets.QComboBox()
        self.bitsCombo.addItems(["5", "6", "7", "8"])
        self.bitsCombo.setCurrentIndex(3)
        self.form2.addRow("Bits", self.bitsCombo)

        self.stopBitsCombo = QtWidgets.QComboBox()
        self.stopBitsCombo.addItems(["1", "1.5", "2"])
        self.form2.addRow("Stop bits", self.stopBitsCombo)

        self.buttonBox = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel, QtCore.Qt.Horizontal, self)
        self.vbox1.addWidget(self.buttonBox)

        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

    def showEvent(self, event):
        import serial.tools.list_ports
        self.portCombo.clear()
        self.portCombo.addItems([comport.device for comport in serial.tools.list_ports.comports()])


class ConnectDialogUdp(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super(ConnectDialogUdp, self).__init__(parent)

        self.setObjectName("ConnectDialogUdp")
        self.setWindowTitle(QtWidgets.QApplication.translate("MainWindow", "Connect (UDP)", None))

        self.vbox1 = QtWidgets.QVBoxLayout(self)

        self.gboxUdp = QtWidgets.QGroupBox(QtWidgets.QApplication.translate("MainWindow", "UDP", None))
        self.vbox1.addWidget(self.gboxUdp)

        self.vbox2 = QtWidgets.QHBoxLayout(self.gboxUdp)

        self.form1 = QtWidgets.QFormLayout()
        self.vbox2.addLayout(self.form1)

        self.host = QtWidgets.QLineEdit("192.168.1.128")
        self.form1.addRow("Host", self.host)

        self.port = QtWidgets.QLineEdit("14000")
        self.form1.addRow("Port", self.port)

        self.buttonBox = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel, QtCore.Qt.Horizontal, self)
        self.vbox1.addWidget(self.buttonBox)

        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)
