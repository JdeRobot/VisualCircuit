from PyQt5.QtCore import Qt,QObject, QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QSplitter, QWidget, QVBoxLayout, QMainWindow, QGridLayout, QLayout,QLabel,QToolBar
from PyQt5.QtGui import QPalette, QColor
from multiprocessing import Process,Queue,Pipe

class Worker(QObject):
    signal = pyqtSignal(str)
    def __init__(self,queue):
            QObject.__init__(self)
            self.queue = queue
    def run(self):
        while True:
            text = self.queue.get()
            self.signal.emit(text)

#<---------------------shared memory worker class-------------->
# class Worker1(QObject):
#     from utils.wires.wire_str import read_string
#     from time import sleep
#     signal = pyqtSignal(str)
#     def __init__(self):
#             QObject.__init__(self)
#     def run(self):
#         while True:
#             txt = self.read_string("ERROR#LOG").get()
#             print(txt)
#             self.signal.emit(txt[0])
#______________________________________________________________>

class Console(QMainWindow,QWidget,QObject):


    def __init__(self,queue):
        super(Console, self).__init__()
        self.resize(800, 600)
        self.queue = queue
#------------------------------------------------------------------------------------#       
        self.console = QVBoxLayout()
        self.toolbar = QToolBar()
        self.toolbar.addWidget(QLabel("Console"))
        self.text_console = QLabel("")
        self.text_console.setAutoFillBackground(True)
        p = self.text_console.palette()
        p.setColor(self.text_console.backgroundRole(), Qt.black)
        self.text_console.setPalette(p)
        self.text_console.setAlignment(Qt.AlignLeft)

        self.console.addWidget(self.toolbar) 
        self.console.addWidget(self.text_console) 
        

        self.left = QWidget()
        self.left.setLayout(self.console)
#------------------------------------------------------------------------------------#        
        self.freqm = QVBoxLayout()
        self.toolbar2 = QToolBar()
        self.toolbar2.addWidget(QLabel("Frequency Monitor"))

        self.text_freq = QLabel("")
        self.text_freq.setAutoFillBackground(True)
        p2 = self.text_freq.palette()
        p2.setColor(self.text_freq.backgroundRole(), Qt.black)
        self.text_freq.setPalette(p2)
        self.text_freq.setAlignment(Qt.AlignLeft)

        self.freqm.addWidget(self.toolbar2) 
        self.freqm.addWidget(self.text_freq) 
        
        self.right = QWidget()
        self.right.setLayout(self.freqm)
#------------------------------------------------------------------------------------#        
        self.splitter = QSplitter(Qt.Horizontal)
        self.splitter.addWidget(self.left)
        self.splitter.addWidget(self.right)

        self.setCentralWidget(self.splitter)
#------------------------------------------------------------------------------------#        

    def loginfo_console(self, msg):
        txt = self.text_console.text()
        self.text_console.setText(txt+"\n"+msg)

    def loginfo_fm(self, msg):
        end = "END"
        txt = self.text_freq.text()
        self.text_freq.setText(txt+"\n"+msg)
        msg= msg.strip()
        if msg == end:
                self.text_freq.setText(" ")

def set_theme():
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.black)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    return palette
    
