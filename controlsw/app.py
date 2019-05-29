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
from common import Valve, PT, ValveControl, PTControl, DIODiagnostics, PTDiagnostics, ConnectDialogSerial

__version__ = '0.0.1'

class MainWindow(QtWidgets.QMainWindow):

    update_dio = QtCore.pyqtSignal(int, int, int)
    update_pt = QtCore.pyqtSignal(int, int, float)

    set_dio = QtCore.pyqtSignal(int, int, int)

    def __init__(self, parent=None, ini="gse.ini"):
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

        self.devid = config['app'].get('devid', 0x80)

        self.valves = []
        self.pt = []

        self.valve_controls = []
        self.pt_controls = []

        self.cols = []

        for s in config.sections():
            if config[s].get('type') == 'valve':
                v = Valve()
                v.name = s
                v.label = config[s].get('label', "Valve")
                v.devid = int(config[s].get('devid', '0'), 0)
                v.channel = int(config[s].get('channel'), 0)
                v.invert = bool(int(config[s].get('invert'), 0))
                self.valves.append(v)

                col = int(config[s].get('col', 0))

                for k in range(col+1-len(self.cols)):
                    vb = QtWidgets.QVBoxLayout()
                    vb.setAlignment(QtCore.Qt.AlignTop)
                    self.cols.append(vb)
                    self.hbox1.addLayout(vb)

                vc = ValveControl()
                vc.setTitle(v.label)
                v.control = vc
                vc.openButton.clicked.connect(partial(self.do_open_valve, v))
                vc.closeButton.clicked.connect(partial(self.do_close_valve, v))
                self.valve_controls.append(vc)
                self.cols[col].addWidget(vc)
            elif config[s].get('type') == 'pt':
                pt = PT()
                pt.name = s
                pt.label = config[s].get('label', "PT")
                pt.devid = int(config[s].get('devid', '0'), 0)
                pt.channel = int(config[s].get('channel'), 0)
                pt.zero = float(config[s].get('zero', 0.0))
                pt.slope = float(config[s].get('slope', 1.0))
                self.pt.append(pt)

                col = int(config[s].get('col', 0))

                for k in range(col+1-len(self.cols)):
                    vb = QtWidgets.QVBoxLayout()
                    vb.setAlignment(QtCore.Qt.AlignTop)
                    self.cols.append(vb)
                    self.hbox1.addLayout(vb)

                ptc = PTControl()
                ptc.setTitle(pt.label)
                pt.control = ptc
                self.pt_controls.append(ptc)
                self.cols[col].addWidget(ptc)

        for c in self.valves:
            c.set_state(None)
        for c in self.pt:
            c.set_value(None)

        self.log_dir = None
        self.serial_log_file = None
        self.pt_log_file = None
        self.valve_log_file = None
        self.thrust_log_file = None

    def do_set_log_dir(self):
        log_dir = str(QtWidgets.QFileDialog.getExistingDirectory(self, "Select Log Directory"))

        if log_dir:
            self.log_dir = log_dir
            self.serial_log_file = open(os.path.join(self.log_dir, "serial.log"), 'w')
            self.pt_log_file = open(os.path.join(self.log_dir, "pt.csv"), 'w')
            self.valve_log_file = open(os.path.join(self.log_dir, "valve.csv"), 'w')
            self.thrust_log_file = open(os.path.join(self.log_dir, "thrust.csv"), 'w')

    def do_connect_serial(self):
        res = self.connectDialog.exec()

        if res:
            self.interface = None
            self.statusResource.setText("Not connected")

            for v in self.valve_controls:
                v.set_status(None)
            for pt in self.pt_controls:
                pt.set_value(None)

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

                self.statusResource.setText(self.interface.port)

                self.interface.raw_log_callback = self.serial_log
            except:
                self.interface = None
                raise

    def do_connect_xbee(self):
        res = self.connectDialog.exec()

        if res:
            self.interface = None
            self.statusResource.setText("Not connected")

            for v in self.valve_controls:
                v.set_status(None)
            for pt in self.pt_controls:
                pt.set_value(None)

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

                self.statusResource.setText(self.interface.port)

                self.interface.raw_log_callback = self.serial_log
            except:
                self.interface = None
                raise

    def do_disconnect(self):
        self.interface = None
        self.statusResource.setText("Not connected")

        for v in self.valve_controls:
            v.set_status(None)
        for pt in self.pt_controls:
            pt.set_value(None)

    def do_dio_diagnostics(self):
        self.DIODiagnostics.show()

    def do_pt_diagnostics(self):
        self.ptDiagnostics.show()

    def do_open_valve(self, valve):
        self.set_dio.emit(valve.devid, valve.channel, 0 if valve.invert else 1)

    def do_close_valve(self, valve):
        self.set_dio.emit(valve.devid, valve.channel, 1 if valve.invert else 0)

    def on_set_dio(self, devid, ch, state):
        if self.interface:
            self.interface.send(packet.Packet(struct.pack('BB', ch, 1 if state else 0), devid, self.devid, 0x00, 0x11))

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

                if isinstance(pkt, packet.DIOStatePacket):
                    # DIO states

                    if self.valve_log_file:
                        self.valve_log_file.write("{},{},{:#x}\n".format(time.time(), pkt.source, pkt.state))

                    for k in range(16):
                        self.update_dio.emit(pkt.source, k, pkt.state & 1 << k)

                    for v in self.valves:
                        if v.devid == pkt.source:
                            v.set_state(pkt.state & (1 << v.channel))

                elif isinstance(pkt, packet.PTReadingPacket):
                    # PT readings
                    val = [x/10000 for x in pkt.values]
                    print(val)

                    if self.pt_log_file:
                        self.pt_log_file.write("{},{},{}\n".format(time.time(), pkt.source, ",".join(["{:.4f}".format(x) for x in val])))

                    for k in range(len(val)):
                        self.update_pt.emit(pkt.source, k, val[k])

                    for pt in self.pt:
                        if pt.devid == pkt.source:
                            pt.set_value(val[pt.channel])

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

