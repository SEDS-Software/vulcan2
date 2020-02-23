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

import sys
import platform

import PyQt5
from PyQt5 import Qt, QtCore, QtGui, QtWidgets

import array
import configparser
import os.path
import struct
import serial
import sys
import time

from functools import partial

import interface, packet
from common import DIOChannel, AnalogChannel, DIOInputControl, DIOOutputControl, AnalogControl, CommandControl, FlightComputerStatusControl, LaunchSequenceControl, DIODiagnostics, PTDiagnostics, ConnectDialogSerial

__version__ = '0.0.1'

class MainWindow(QtWidgets.QMainWindow):

    rx_pkt = QtCore.pyqtSignal(packet.Packet)

    update_dio = QtCore.pyqtSignal(int, int, int, int)
    update_analog = QtCore.pyqtSignal(int, int, int, float)
    update_pt = QtCore.pyqtSignal(int, int, float)

    set_dio = QtCore.pyqtSignal(int, int, int, int)

    def __init__(self, parent=None, ini=["channels.ini", "gse.ini"]):
        super(MainWindow, self).__init__(parent)

        self.ini = ini

        self.devid = 0x80

        self.pkt_buffer = bytearray()

        self.setObjectName("MainWindow")
        self.setWindowTitle(QtWidgets.QApplication.translate("MainWindow", "SEDS Vulcan II Control Software", None))

        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.setCentralWidget(self.centralwidget)

        self.vbox1 = QtWidgets.QVBoxLayout(self.centralwidget)

        # menu bar
        self.menubar = QtWidgets.QMenuBar(self)
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuFile.setTitle(QtWidgets.QApplication.translate("MainWindow", "&File", None))
        self.menubar.addAction(self.menuFile.menuAction())
        self.menuConfigure = QtWidgets.QMenu(self.menubar)
        self.menuConfigure.setObjectName("menuConfigure")
        self.menuConfigure.setTitle(QtWidgets.QApplication.translate("MainWindow", "Configure", None))
        self.menubar.addAction(self.menuConfigure.menuAction())
        self.menuTools = QtWidgets.QMenu(self.menubar)
        self.menuTools.setObjectName("menuTools")
        self.menuTools.setTitle(QtWidgets.QApplication.translate("MainWindow", "Tools", None))
        self.menubar.addAction(self.menuTools.menuAction())
        self.menuHelp = QtWidgets.QMenu(self.menubar)
        self.menuHelp.setObjectName("menuHelp")
        self.menuHelp.setTitle(QtWidgets.QApplication.translate("MainWindow", "&Help", None))
        self.menubar.addAction(self.menuHelp.menuAction())
        self.setMenuBar(self.menubar)

        self.actionLogDir = QtWidgets.QAction(self)
        self.actionLogDir.setObjectName("actionLogDir")
        self.actionLogDir.setIcon(QtGui.QIcon.fromTheme("document-save"))
        self.actionLogDir.setText(QtWidgets.QApplication.translate("MainWindow", "&Log Directory", None))
        self.actionLogDir.triggered.connect(self.do_set_log_dir)
        self.menuFile.addAction(self.actionLogDir)

        self.actionQuit = QtWidgets.QAction(self)
        self.actionQuit.setObjectName("actionQuit")
        self.actionQuit.setIcon(QtGui.QIcon.fromTheme("application-exit"))
        self.actionQuit.setText(QtWidgets.QApplication.translate("MainWindow", "&Quit", None))
        self.actionQuit.triggered.connect(QtCore.QCoreApplication.instance().quit)
        self.menuFile.addAction(self.actionQuit)

        self.actionConnectSerial = QtWidgets.QAction(self)
        self.actionConnectSerial.setObjectName("actionConnectSerial")
        self.actionConnectSerial.setText(QtWidgets.QApplication.translate("MainWindow", "&Connect (Serial)", None))
        self.actionConnectSerial.triggered.connect(self.do_connect_serial)
        self.menuConfigure.addAction(self.actionConnectSerial)

        self.actionConnectXBee = QtWidgets.QAction(self)
        self.actionConnectXBee.setObjectName("actionConnectXBee")
        self.actionConnectXBee.setText(QtWidgets.QApplication.translate("MainWindow", "&Connect (XBee)", None))
        self.actionConnectXBee.triggered.connect(self.do_connect_xbee)
        self.menuConfigure.addAction(self.actionConnectXBee)

        self.actionDisconnect = QtWidgets.QAction(self)
        self.actionDisconnect.setObjectName("actionDisconnect")
        self.actionDisconnect.setText(QtWidgets.QApplication.translate("MainWindow", "&Disconnect", None))
        self.actionDisconnect.triggered.connect(self.do_disconnect)
        self.menuConfigure.addAction(self.actionDisconnect)

        self.actionDIODiagnostics = QtWidgets.QAction(self)
        self.actionDIODiagnostics.setObjectName("actionDIODiagnostics")
        self.actionDIODiagnostics.setText(QtWidgets.QApplication.translate("MainWindow", "&DIO Diagnostics", None))
        self.actionDIODiagnostics.triggered.connect(self.do_dio_diagnostics)
        self.menuTools.addAction(self.actionDIODiagnostics)

        self.actionPTDiagnostics = QtWidgets.QAction(self)
        self.actionPTDiagnostics.setObjectName("actionPTDiagnostics")
        self.actionPTDiagnostics.setText(QtWidgets.QApplication.translate("MainWindow", "&PT Diagnostics", None))
        self.actionPTDiagnostics.triggered.connect(self.do_pt_diagnostics)
        self.menuTools.addAction(self.actionPTDiagnostics)

        self.actionAbout = QtWidgets.QAction(self)
        self.actionAbout.setObjectName("actionAbout")
        self.actionAbout.setText(QtWidgets.QApplication.translate("MainWindow", "About", None))
        self.actionAbout.triggered.connect(self.do_about)
        self.menuHelp.addAction(self.actionAbout)

        # status bar
        self.statusbar = QtWidgets.QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)

        self.statusResource = QtWidgets.QLabel()
        self.statusResource.setText("Not connected")
        self.statusbar.addWidget(self.statusResource)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)

        self.connectDialog = ConnectDialogSerial(self)

        self.DIODiagnostics = DIODiagnostics(self)
        self.update_dio.connect(self.DIODiagnostics.on_update_dio)
        self.DIODiagnostics.set_dio.connect(self.on_set_dio)
        self.ptDiagnostics = PTDiagnostics(self)
        self.update_pt.connect(self.ptDiagnostics.on_update_pt)

        self.set_dio.connect(self.on_set_dio)

        self.interface = None

        self.hbox1 = QtWidgets.QHBoxLayout()
        self.vbox1.addLayout(self.hbox1)

        config = configparser.ConfigParser()
        config.read(self.ini)

        if config['app'].get('title'):
            self.setWindowTitle(config['app'].get('title'))

        self.devid = int(config['app'].get('devid', "0x80"), 0)

        self.dio_channels = {}
        self.analog_channels = {}

        self.controls = []

        self.cols = []

        # build channel objects
        for s in config.sections():
            if config[s].get('type') in ['dio', 'dio_in', 'dio_out', 'valve']:
                ch = DIOChannel()
                ch.name = s
                ch.label = config[s].get('label', s)
                ch.bank = int(config[s].get('bank', '0'), 0)
                ch.devid = int(config[s].get('devid', '0'), 0)
                ch.channel = int(config[s].get('channel'), 0)
                ch.invert = bool(int(config[s].get('invert'), 0))
                if config[s].get('type') == 'valve':
                    ch.enable_label = config[s].get('enable_label', "Open")
                    ch.disable_label = config[s].get('disable_label', "Close")
                    ch.enabled_label = config[s].get('enabled_label', "Open")
                    ch.disabled_label = config[s].get('disabled_label', "Closed")
                else:
                    ch.enable_label = config[s].get('enable_label', "Enable")
                    ch.disable_label = config[s].get('disable_label', "Disable")
                    ch.enabled_label = config[s].get('enabled_label', "Enabled")
                    ch.disabled_label = config[s].get('disabled_label', "Disabled")
                ch.timeout = float(config[s].get('timeout', 1.0))
                ch.send_pkt = self.send_pkt
                self.dio_channels[s] = ch

            if config[s].get('type') in ['analog', 'pt']:
                ch = AnalogChannel()
                ch.name = s
                ch.label = config[s].get('label', s)
                ch.bank = int(config[s].get('bank', '0'), 0)
                ch.devid = int(config[s].get('devid', '0'), 0)
                ch.channel = int(config[s].get('channel'), 0)
                ch.zero = float(config[s].get('zero', 0.0))
                ch.slope = float(config[s].get('slope', 1.0))
                if config[s].get('type') == 'pt':
                    ch.unit = config[s].get('unit', "PSI")
                else:
                    ch.unit = config[s].get('unit', "V")
                ch.format = config[s].get('format', "{:6.2f} {}")
                ch.unknown_format = config[s].get('unknown_format', "---.-- {}")
                ch.timeout = float(config[s].get('timeout', 1.0))
                self.analog_channels[s] = ch

        # build UI
        for s in config.sections():
            col = int(config[s].get('col', -1))

            if col < 0:
                continue

            # add columns
            for k in range(col+1-len(self.cols)):
                vb = QtWidgets.QVBoxLayout()
                vb.setAlignment(QtCore.Qt.AlignTop)
                self.cols.append(vb)
                self.hbox1.addLayout(vb)

            # add control
            if config[s].get('type') in ['dio', 'dio_out', 'valve']:
                c = DIOOutputControl(self.dio_channels[s])
                c.sizePolicy().setHorizontalStretch(1)
                self.controls.append(c)
                self.cols[col].addWidget(c)

            elif config[s].get('type') == 'dio_in':
                c = DIOInputControl(self.dio_channels[s])
                c.sizePolicy().setHorizontalStretch(1)
                self.controls.append(c)
                self.cols[col].addWidget(c)

            elif config[s].get('type') in ['analog', 'pt']:
                c = AnalogControl(self.analog_channels[s])
                c.sizePolicy().setHorizontalStretch(1)
                self.controls.append(c)
                self.cols[col].addWidget(c)

            elif config[s].get('type') == 'cmd':
                c = CommandControl()
                c.sizePolicy().setHorizontalStretch(1)
                c.set_name(config[s].get('label', "Command"))
                c.devid = int(config[s].get('devid', '0'), 0)
                c.cmd = int(config[s].get('cmd', '0'), 0)
                c.data = bytes.fromhex(config[s].get('data', ''))
                c.sendButton.clicked.connect(partial(self.do_send_cmd, int(config[s].get('devid', '0'), 0), int(config[s].get('cmd', '0'), 0), bytes.fromhex(config[s].get('data', ''))))
                self.controls.append(c)
                self.cols[col].addWidget(c)

            elif config[s].get('type') == 'flightcomputer':
                c = FlightComputerStatusControl(gps=int(config[s].get('gps', 1)))
                c.sizePolicy().setHorizontalStretch(1)
                c.set_name(config[s].get('label', "Flight Computer Status"))
                c.devid = int(config[s].get('devid', '0'), 0)
                self.rx_pkt.connect(c.handle_packet)
                self.controls.append(c)
                self.cols[col].addWidget(c)

            elif config[s].get('type') == 'launchsequence':
                c = LaunchSequenceControl(self)
                c.sizePolicy().setHorizontalStretch(1)
                c.set_name(config[s].get('label', "Launch"))
                c.devid = int(config[s].get('devid', str(self.devid)), 0)
                self.rx_pkt.connect(c.handle_packet)
                self.controls.append(c)
                self.cols[col].addWidget(c)


        for ch in self.dio_channels.values():
            ch.set_raw_value(None)
        for ch in self.analog_channels.values():
            ch.set_raw_value(None)

        self.log_dir = None
        self.serial_log_file = None
        self.pt_log_file = None
        self.valve_log_file = None
        self.gps_log_file = None
        self.thrust_log_file = None

    def do_set_log_dir(self):
        log_dir = str(QtWidgets.QFileDialog.getExistingDirectory(self, "Select Log Directory"))

        if log_dir:
            self.log_dir = log_dir
            self.serial_log_file = open(os.path.join(self.log_dir, "serial.log"), 'w')
            self.pt_log_file = open(os.path.join(self.log_dir, "pt.csv"), 'w')
            self.valve_log_file = open(os.path.join(self.log_dir, "valve.csv"), 'w')
            self.gps_log_file = open(os.path.join(self.log_dir, "gps.csv"), 'w')
            self.thrust_log_file = open(os.path.join(self.log_dir, "thrust.csv"), 'w')

    def do_connect_serial(self):
        res = self.connectDialog.exec()

        if res:
            self.interface = None
            self.statusResource.setText("Not connected")

            for ch in self.dio_channels.values():
                ch.set_raw_value(None)
            for ch in self.analog_channels.values():
                ch.set_raw_value(None)

            try:
                self.interface = interface.SerialInterface(self.connectDialog.portCombo.currentText(), int(self.connectDialog.speedCombo.currentText()))

                if self.connectDialog.flowCombo.currentIndex() == 0:
                    self.interface.serial_port.xonxoff = False
                    self.interface.serial_port.rtscts = False
                elif self.connectDialog.flowCombo.currentIndex() == 1:
                    self.interface.serial_port.xonxoff = False
                    self.interface.serial_port.rtscts = True
                elif self.connectDialog.flowCombo.currentIndex() == 2:
                    self.interface.serial_port.xonxoff = True
                    self.interface.serial_port.rtscts = False

                if self.connectDialog.parityCombo.currentIndex() == 0:
                    self.interface.serial_port.parity = serial.PARITY_NONE
                elif self.connectDialog.parityCombo.currentIndex() == 1:
                    self.interface.serial_port.parity = serial.PARITY_EVEN
                elif self.connectDialog.parityCombo.currentIndex() == 2:
                    self.interface.serial_port.parity = serial.PARITY_ODD

                if self.connectDialog.bitsCombo.currentIndex() == 0:
                    self.interface.serial_port.bytesize = serial.FIVEBITS
                elif self.connectDialog.bitsCombo.currentIndex() == 1:
                    self.interface.serial_port.bytesize = serial.SIXBITS
                elif self.connectDialog.bitsCombo.currentIndex() == 2:
                    self.interface.serial_port.bytesize = serial.SEVENBITS
                elif self.connectDialog.bitsCombo.currentIndex() == 3:
                    self.interface.serial_port.bytesize = serial.EIGHTBITS

                if self.connectDialog.stopBitsCombo.currentIndex() == 0:
                    self.interface.serial_port.stopbits = serial.STOPBITS_ONE
                elif self.connectDialog.stopBitsCombo.currentIndex() == 1:
                    self.interface.serial_port.stopbits = serial.STOPBITS_ONE_POINT_FIVE
                elif self.connectDialog.stopBitsCombo.currentIndex() == 2:
                    self.interface.serial_port.stopbits = serial.STOPBITS_TWO

                self.statusResource.setText("{} / TX pkts {} / RX pkts {} / RX errs {}".format(self.interface.port, self.interface.tx_pkts, self.interface.rx_pkts, self.interface.rx_errs))

                self.interface.raw_log_callback = self.serial_log
            except:
                self.interface = None
                raise

    def do_connect_xbee(self):
        res = self.connectDialog.exec()

        if res:
            self.interface = None
            self.statusResource.setText("Not connected")

            for ch in self.dio_channels.values():
                ch.set_raw_value(None)
            for ch in self.analog_channels.values():
                ch.set_raw_value(None)

            try:
                self.interface = interface.XBeeInterface(self.connectDialog.portCombo.currentText(), int(self.connectDialog.speedCombo.currentText()))

                if self.connectDialog.flowCombo.currentIndex() == 0:
                    self.interface.serial_port.xonxoff = False
                    self.interface.serial_port.rtscts = False
                elif self.connectDialog.flowCombo.currentIndex() == 1:
                    self.interface.serial_port.xonxoff = False
                    self.interface.serial_port.rtscts = True
                elif self.connectDialog.flowCombo.currentIndex() == 2:
                    self.interface.serial_port.xonxoff = True
                    self.interface.serial_port.rtscts = False

                if self.connectDialog.parityCombo.currentIndex() == 0:
                    self.interface.serial_port.parity = serial.PARITY_NONE
                elif self.connectDialog.parityCombo.currentIndex() == 1:
                    self.interface.serial_port.parity = serial.PARITY_EVEN
                elif self.connectDialog.parityCombo.currentIndex() == 2:
                    self.interface.serial_port.parity = serial.PARITY_ODD

                if self.connectDialog.bitsCombo.currentIndex() == 0:
                    self.interface.serial_port.bytesize = serial.FIVEBITS
                elif self.connectDialog.bitsCombo.currentIndex() == 1:
                    self.interface.serial_port.bytesize = serial.SIXBITS
                elif self.connectDialog.bitsCombo.currentIndex() == 2:
                    self.interface.serial_port.bytesize = serial.SEVENBITS
                elif self.connectDialog.bitsCombo.currentIndex() == 3:
                    self.interface.serial_port.bytesize = serial.EIGHTBITS

                if self.connectDialog.stopBitsCombo.currentIndex() == 0:
                    self.interface.serial_port.stopbits = serial.STOPBITS_ONE
                elif self.connectDialog.stopBitsCombo.currentIndex() == 1:
                    self.interface.serial_port.stopbits = serial.STOPBITS_ONE_POINT_FIVE
                elif self.connectDialog.stopBitsCombo.currentIndex() == 2:
                    self.interface.serial_port.stopbits = serial.STOPBITS_TWO

                self.statusResource.setText("{} / TX pkts {} / RX pkts {} / RX errs {} / RSSI {} dBm".format(self.interface.port, self.interface.tx_pkts, self.interface.rx_pkts, self.interface.rx_errs, self.interface.rssi))

                self.interface.raw_log_callback = self.serial_log
            except:
                self.interface = None
                raise

    def do_disconnect(self):
        self.interface = None
        self.statusResource.setText("Not connected")

        for ch in self.dio_channels.values():
            ch.set_raw_value(None)
        for ch in self.analog_channels.values():
            ch.set_raw_value(None)

    def do_dio_diagnostics(self):
        self.DIODiagnostics.show()

    def do_pt_diagnostics(self):
        self.ptDiagnostics.show()

    def do_open_valve(self, valve):
        self.set_dio.emit(valve.devid, valve.bank, valve.channel, 0 if valve.invert else 1)

    def do_close_valve(self, valve):
        self.set_dio.emit(valve.devid, valve.bank, valve.channel, 1 if valve.invert else 0)

    def on_set_dio(self, devid, bank, ch, state):
        if self.interface:
            pkt = packet.DIOSetBitPacket()
            pkt.dest = devid
            pkt.source = self.devid
            pkt.flags = 0
            pkt.bank = bank
            pkt.bit = ch
            pkt.state = state
            self.interface.send(pkt)

    def do_send_cmd(self, devid, cmd, data):
        if self.interface:
            pkt = packet.CommandPacket()
            pkt.dest = devid
            pkt.source = self.devid
            pkt.flags = 0
            pkt.cmd = cmd
            pkt.data = data
            self.interface.send(pkt)

    def send_pkt(self, pkt):
        if self.interface:
            pkt.source = self.devid
            self.interface.send(pkt)

    def serial_log(self, tx, data):
        if self.serial_log_file:
            self.serial_log_file.write("{} {} {}\n".format(time.time(), 'TX' if tx else 'RX', data.hex()))

    def tick(self):
        if self.interface:
            while True:
                pkt = self.interface.poll()

                if pkt is None:
                    break

                print(pkt)

                self.rx_pkt.emit(pkt)

                if isinstance(pkt, packet.DIOStatePacket):
                    # DIO states

                    if self.valve_log_file:
                        self.valve_log_file.write("{},{},{:#x}\n".format(time.time(), pkt.source, pkt.state))

                    for k in range(16):
                        self.update_dio.emit(pkt.source, pkt.bank, k, pkt.state & 1 << k)

                    for ch in self.dio_channels.values():
                        if ch.devid == pkt.source and ch.bank == pkt.bank:
                            ch.set_raw_value(pkt.state & (1 << ch.channel))

                elif isinstance(pkt, packet.AnalogValuePacket):
                    # PT readings
                    val = [x/10000 for x in pkt.values]
                    print(val)

                    if self.pt_log_file:
                        self.pt_log_file.write("{},{},{}\n".format(time.time(), pkt.source, ",".join(["{:.4f}".format(x) for x in val])))

                    for k in range(len(val)):
                        self.update_analog.emit(pkt.source, pkt.bank, k, val[k])
                        self.update_pt.emit(pkt.source, k, val[k])

                    for ch in self.analog_channels.values():
                        if ch.devid == pkt.source and ch.bank == pkt.bank:
                            ch.set_raw_value(val[ch.channel])

                elif isinstance(pkt, packet.GpsPositionPacket):
                    # GPS position

                    if self.gps_log_file:
                        self.gps_log_file.write("{},{},{},{},{},{},{},{},{},{},{},{}\n".format(time.time(), pkt.source, pkt.latitude, pkt.longitude, pkt.altitude, pkt.speed, pkt.heading, pkt.satellites, pkt.fix_type, pkt.date, pkt.time, pkt.hdop))

            if isinstance(self.interface, interface.XBeeInterface):
                self.statusResource.setText("{} / TX pkts {} / RX pkts {} / RX errs {} / RSSI {} dBm".format(self.interface.port, self.interface.tx_pkts, self.interface.rx_pkts, self.interface.rx_errs, self.interface.rssi))
            else:
                self.statusResource.setText("{} / TX pkts {} / RX pkts {} / RX errs {}".format(self.interface.port, self.interface.tx_pkts, self.interface.rx_pkts, self.interface.rx_errs))

        for c in self.dio_channels.values():
            if hasattr(c, 'tick'):
                c.tick()

        for c in self.analog_channels.values():
            if hasattr(c, 'tick'):
                c.tick()

        for c in self.controls:
            if hasattr(c, 'tick'):
                c.tick()

    def do_about(self):
        QtWidgets.QMessageBox.about(self, "About SEDS Vulcan II Control Software",
            """<b>SEDS Vulcan II Control Software</b> v {}
<p>Copyright &copy; 2019 Alex Forencich</p>
<p>MIT license</p>
<p>Python {}</p>
<p>PyQt5 {}</p>
<p>Qt {}</p>
<p>{} {}</p>""".format(__version__,
        platform.python_version(), PyQt5.Qt.PYQT_VERSION_STR, PyQt5.QtCore.QT_VERSION_STR,
        platform.system(), platform.release()))

