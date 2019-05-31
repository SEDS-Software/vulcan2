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

import serial

from functools import partial


class Valve(object):
    def __init__(self):
        self.name = "v"
        self.label = "Valve"
        self.devid = 0x00
        self.bank = 0
        self.channel = 0
        self.invert = False
        self.control = None

    def set_state(self, state):
        if self.control is None:
            return

        if self.control.detailsLabel:
            if state is None:
                s = "-"
            else:
                s = "{:d}".format(bool(state))
            if self.invert:
                s += " Inv"
            self.control.detailsLabel.setText("Dev {:#04x}  Ch {:d}  Raw {}".format(self.devid, self.channel, s))

        if state is None:
            self.control.set_status(None)
        elif self.invert:
            self.control.set_status('closed' if state else 'open')
        else:
            self.control.set_status('open' if state else 'closed')


class PT(object):
    def __init__(self):
        self.name = "pt"
        self.label = "PT"
        self.devid = 0x00
        self.channel = 0
        self.zero = 0.0
        self.slope = 1.0
        self.control = None

    def convert_value(self, value):
        return (value - self.zero) * self.slope

    def set_value(self, value):
        if self.control is None:
            return

        if self.control.detailsLabel:
            if value is None:
                s = "-.----"
            else:
                s = "{:1.4f}".format(value)
            self.control.detailsLabel.setText("Dev {:#04x}  Ch {:d}  Raw {} v".format(self.devid, self.channel, s))

        if value is None:
            self.control.set_value(None)
        else:
            self.control.set_value(self.convert_value(value))


class ValveControl(QtWidgets.QGroupBox):
    def __init__(self):
        self.name = "Valve"
        self.status = None

        super(ValveControl, self).__init__()

        self.setTitle(QtWidgets.QApplication.translate("MainWindow", "Valve", None))

        self.vbox1 = QtWidgets.QVBoxLayout(self)

        self.hbox1 = QtWidgets.QHBoxLayout()
        self.vbox1.addLayout(self.hbox1)

        self.vbox2 = QtWidgets.QVBoxLayout()
        self.hbox1.addLayout(self.vbox2)

        self.openButton = QtWidgets.QPushButton(QtWidgets.QApplication.translate("MainWindow", "Open", None), self)
        self.vbox2.addWidget(self.openButton)

        self.closeButton = QtWidgets.QPushButton(QtWidgets.QApplication.translate("MainWindow", "Close", None), self)
        self.vbox2.addWidget(self.closeButton)

        self.statusLabel = QtWidgets.QLabel("Unknown")
        self.statusLabel.setStyleSheet('background-color: silver')
        self.statusLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.hbox1.addWidget(self.statusLabel, 0)

        self.detailsLabel = QtWidgets.QLabel("Dev 0x00  Ch 0  Raw -")
        self.vbox1.setContentsMargins(16, 16, 16, 8)
        self.vbox1.addWidget(self.detailsLabel)

        self.set_status(None)

        self.openButton.clicked.connect(self.do_open)
        self.closeButton.clicked.connect(self.do_close)

    def set_name(self, name):
        self.name = name
        self.setTitle(name)

    def set_status(self, val):
        self.status = val
        if val == 'open':
            self.statusLabel.setText("Open")
            self.statusLabel.setStyleSheet('background-color: green; color: white')
        elif val == 'closed':
            self.statusLabel.setText("Closed")
            self.statusLabel.setStyleSheet('background-color: red; color: white')
        else:
            self.status = None
            self.statusLabel.setText("Unknown")
            self.statusLabel.setStyleSheet('background-color: silver')

    def do_open(self):
        pass

    def do_close(self):
        pass


class PTControl(QtWidgets.QGroupBox):
    def __init__(self):
        self.name = "Pressure"
        self.value = None
        self.format = "{:3.2f} {}"
        self.unknown_format = "---.-- {}"
        self.unit = "PSI"

        super(PTControl, self).__init__()

        self.setTitle(QtWidgets.QApplication.translate("MainWindow", "Pressure", None))

        self.vbox1 = QtWidgets.QVBoxLayout(self)

        self.statusLabel = QtWidgets.QLabel("-.--- PSI")
        font = QtGui.QFont("monospace")
        font.setPointSize(20)
        font.setStyleHint(QtGui.QFont.TypeWriter)
        self.statusLabel.setFont(font)
        self.statusLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.vbox1.addWidget(self.statusLabel, 0)

        self.detailsLabel = QtWidgets.QLabel("Dev 0x00  Ch 0  Raw -.---- v")
        self.vbox1.setContentsMargins(16, 16, 16, 8)
        self.vbox1.addWidget(self.detailsLabel)

        self.set_value(None)

    def set_value(self, val):
        try:
            self.val = float(val)
            self.statusLabel.setText(self.format.format(self.val, self.unit))
            self.statusLabel.setStyleSheet('')
        except:
            self.val = None
            self.statusLabel.setText(self.unknown_format.format(self.unit))
            self.statusLabel.setStyleSheet('background-color: silver')


class DIODiagnostics(QtWidgets.QDialog):
    
    set_dio = QtCore.pyqtSignal(int, int, int)

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

