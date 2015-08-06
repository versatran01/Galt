#!/usr/bin/python3

import sys
from PyQt5.QtCore import (QDir, Qt, QPoint, QRect, QSize)
from PyQt5.QtWidgets import (QWidget, QLabel, QApplication, QMainWindow,
                             QAction, QFileDialog, qApp, QMessageBox, QMenu,
                             QSizePolicy, QScrollArea, QToolTip)
from PyQt5.QtGui import (QPixmap, QIcon, QImage, QPalette, QPainter, QPen,
                         QImageWriter, qRgb, qRgba)


class Labeler(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initConsts()
        self.initUI()

    def initConsts(self):
        self.factorStep = 0.25
        self.maxFactor = 4.0
        self.minFactor = 1 / self.maxFactor
        self.scaleFactor = 0.0
        self.appName = 'Apple Labeler'
        self.resourceDir = 'resources'
        self.iconDir = self.resourceDir + '/icons'

    def initUI(self):

        # TODO: move these to a method
        self.imageLabel = QLabel()
        self.imageLabel.setBackgroundRole(QPalette.Base)
        self.imageLabel.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.imageLabel.setScaledContents(True)

        self.scrollArea = QScrollArea()
        self.scrollArea.setBackgroundRole(QPalette.Dark)
        self.scrollArea.setWidget(self.imageLabel)
        self.setCentralWidget(self.scrollArea)

        self.createActions()
        self.createMenus()
        self.createToolbar()
        self.createWindow()

        self.show()

    def aboutThisApp(self):
        QMessageBox.about(self, 'About Apple Labeler',
                          '<b>Apple Labeler</b> is an apple labeling tool.')

    def resetScaleFactor(self):
        self.scaleFactor = 1.0

    def createToolbar(self):
        # View
        self.toolbar = self.addToolBar('Zoom In')
        self.toolbar.addAction(self.openAct)
        self.toolbar.addAction(self.saveAct)
        self.toolbar.addSeparator()
        self.toolbar.addAction(self.zoomInAct)
        self.toolbar.addAction(self.zoomOutAct)
        self.toolbar.addAction(self.normalSizeAct)
        self.toolbar.addAction(self.fitToWindowAct)
        self.toolbar.addSeparator()

    def createWindow(self):
        self.setWindowTitle(self.appName)
        self.setWindowIcon(QIcon(self.iconDir + '/apple-512.png'))
        self.centerWindow()

    def getIcon(self, file):
        return self.iconDir + '/' + file

    def createActions(self):
        # File
        # Open
        openIcon = QIcon(self.getIcon('open-file.png'))
        self.openAct = QAction(openIcon, '&Open', self)
        self.openAct.setShortcut('Ctrl+O')
        self.openAct.triggered.connect(self.openFile)

        # Save
        saveIcon =QIcon(self.getIcon('save-file.png'))
        self.saveAct = QAction(saveIcon, '&Save', self)
        self.saveAct.setShortcut('Ctrl+S')
        self.saveAct.setEnabled(False)

        # Exit
        self.exitAct = QAction('&Exit', self)
        self.exitAct.setShortcut('Ctrl+Q')
        self.exitAct.triggered.connect(self.close)

        # View
        # Zoom In
        zoomInIcon = QIcon(self.iconDir + '/zoom-in.png')
        self.zoomInAct = QAction(zoomInIcon, 'Zoom &In', self)
        self.zoomInAct.setShortcut('Ctrl++')
        self.zoomInAct.setEnabled(False)
        self.zoomInAct.triggered.connect(self.zoomIn)

        # Zoom Out
        zoomOutIcon = QIcon(self.iconDir + '/zoom-out.png')
        self.zoomOutAct = QAction(zoomOutIcon, 'Zoom &Out', self)
        self.zoomOutAct.setShortcut('Ctrl+-')
        self.zoomOutAct.setEnabled(False)
        self.zoomOutAct.triggered.connect(self.zoomOut)

        # Normal Size
        normalSizeIcon = QIcon(self.iconDir + '/normal-size.png')
        self.normalSizeAct = QAction(normalSizeIcon, '&Normal Size', self)
        self.normalSizeAct.setShortcut('Ctrl+0')
        self.normalSizeAct.setEnabled(False)
        self.normalSizeAct.triggered.connect(self.normalSize)

        # Fit to Window
        fitToWindowIcon = QIcon(self.iconDir + '/fit-to-window.png')
        self.fitToWindowAct = QAction(fitToWindowIcon, '&Fit to Window', self)
        self.fitToWindowAct.setShortcut('Ctrl+F')
        self.fitToWindowAct.setEnabled(False)
        self.fitToWindowAct.setCheckable(True)
        self.fitToWindowAct.triggered.connect(self.fitToWindow)

        # Help
        # About
        self.aboutAct = QAction('&About', self)
        self.aboutAct.triggered.connect(self.aboutThisApp)

        # About Qt
        self.aboutQtAct = QAction('About &Qt', self)
        self.aboutQtAct.triggered.connect(QApplication.instance().aboutQt)

    def createMenus(self):
        # File
        fileMenu = QMenu('&File', self)
        fileMenu.addAction(self.openAct)
        fileMenu.addAction(self.saveAct)
        fileMenu.addSeparator()
        fileMenu.addAction(self.exitAct)

        # View
        viewMenu = QMenu('&View', self)
        viewMenu.addAction(self.zoomInAct)
        viewMenu.addAction(self.zoomOutAct)
        viewMenu.addAction(self.normalSizeAct)
        viewMenu.addAction(self.fitToWindowAct)

        # help
        helpMenu = QMenu('&Help', self)
        helpMenu.addAction(self.aboutAct)
        helpMenu.addAction(self.aboutQtAct)

        menuBar = self.menuBar()
        menuBar.addMenu(fileMenu)
        menuBar.addMenu(viewMenu)
        menuBar.addMenu(helpMenu)

    def centerWindow(self):
        frame = self.frameGeometry()
        desktop = QApplication.desktop()
        screen = desktop.screenNumber(desktop.cursor().pos())
        centerPoint = desktop.screenGeometry(screen).center()
        frame.moveCenter(centerPoint)
        self.move(frame.topLeft())
        self.resize(640, 480)

    def zoomIn(self):
        self.scaleImage(1 + self.factorStep)

    def zoomOut(self):
        self.scaleImage(1 - self.factorStep)

    def normalSize(self):
        self.imageLabel.adjustSize()
        self.resetScaleFactor()
        self.updateViewActions()

    def fitToWindow(self):
        fitToWindow = self.fitToWindowAct.isChecked()
        self.scrollArea.setWidgetResizable(fitToWindow)
        if not fitToWindow:
            self.normalSize()

        self.updateViewActions()

    def updateViewActions(self):
        isFitToWindowChecked = self.fitToWindowAct.isChecked()
        self.zoomInAct.setEnabled(not isFitToWindowChecked)
        self.zoomOutAct.setEnabled(not isFitToWindowChecked)
        self.normalSizeAct.setEnabled(not isFitToWindowChecked)

    def scaleImage(self, factor):
        self.scaleFactor *= factor
        self.imageLabel.resize(
            self.scaleFactor * self.imageLabel.pixmap().size())

        self.adjustScrollBar(self.scrollArea.horizontalScrollBar(), factor)
        self.adjustScrollBar(self.scrollArea.verticalScrollBar(), factor)

        self.zoomInAct.setEnabled(self.scaleFactor < self.maxFactor)
        self.zoomOutAct.setEnabled(self.scaleFactor > self.minFactor)

    def adjustScrollBar(self, scrollBar, factor):
        scrollBarValue = factor * scrollBar.value() + (factor - 1) * \
                                                      scrollBar.pageStep() / 2
        scrollBar.setValue(int(scrollBarValue))

    def openFile(self):
        fileName, _ = QFileDialog.getOpenFileName(self, 'Open file',
                                                  QDir.currentPath())

        if fileName:
            image = QImage(fileName)

            if image.isNull():
                QMessageBox.information(self, self.appName, "Cannot load "
                                                            "%s." % fileName)
                return

            self.imageLabel.setPixmap(QPixmap.fromImage(image))
            self.resetScaleFactor()

            self.fitToWindowAct.setEnabled(True)
            self.updateViewActions()

            if not self.fitToWindowAct.isChecked():
                self.imageLabel.adjustSize()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    labeler = Labeler()
    sys.exit(app.exec_())
