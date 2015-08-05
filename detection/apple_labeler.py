#!/usr/bin/python3

import sys
import os
from PyQt5.QtCore import QDir, Qt
from PyQt5.QtWidgets import (QWidget, QLabel, QApplication, QMainWindow,
                             QAction, QFileDialog, qApp)
from PyQt5.QtGui import QPixmap, QIcon


class Labeler(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        exitAction = QAction(QIcon(), 'Exit', self)
        exitAction.setShortcut('Ctrl+Q')
        exitAction.triggered.connect(qApp.quit)

        openFile = QAction(QIcon(), 'Open', self)
        openFile.setShortcut('Ctrl+O')
        openFile.setStatusTip('Open new image')
        openFile.triggered.connect(self.showOpenFileDialog)

        # Menu Bar
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        fileMenu.addAction(openFile)
        fileMenu.addAction(exitAction)


        # Tool Bar

        # Status Bar
        self.statusBar()

        self.setWindowTitle('Apple Labeler')
        self.centerWindow()
        self.show()

    def centerWindow(self):
        frame = self.frameGeometry()
        desktop = QApplication.desktop()
        screen = desktop.screenNumber(desktop.cursor().pos())
        centerPoint = desktop.screenGeometry(screen).center()
        frame.moveCenter(centerPoint)
        self.move(frame.topLeft())
        self.resize(640, 480)

    def showOpenFileDialog(self):
        fname, _ = QFileDialog.getOpenFileName(self, 'Open file', os.getcwd())

        self.statusBar().showMessage(fname)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    labeler = Labeler()
    sys.exit(app.exec_())
