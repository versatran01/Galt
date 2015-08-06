#!/usr/bin/python3

import sys
import logging
from PyQt5.QtCore import (QDir, Qt, QPoint, QRect, QSize)
from PyQt5.QtWidgets import (QWidget, QLabel, QApplication, QMainWindow,
                             QAction, QFileDialog, qApp, QMessageBox, QMenu,
                             QSizePolicy, QScrollArea, QToolTip,
                             QGraphicsScene, QGraphicsView, QGraphicsItem,
                             QGraphicsPixmapItem)
from PyQt5.QtGui import (QPixmap, QIcon, QImage, QPalette, QPainter, QPen,
                         QImageWriter, qRgb, qRgba)


class Label(object):
    def __init__(self):
        pass


class ZoomView(QGraphicsView):
    """
    A zoomed view that looks at where the mouse is
    """

    def __init__(self):
        pass


class LabelView(QGraphicsView):
    """
    Main view that looks at the scene
    """

    def __init__(self):
        super().__init__()

    def scaleBy(self, factor):
        self.scale(factor, factor)


class LabelScene(QGraphicsScene):
    def __init__(self):
        super().__init__()

        self._modified = False

    def getModified(self):
        return self._modified

    def setModified(self, modified):
        self._modified = modified

    modified = property(getModified, setModified)

    def loadImage(self, fileName):
        image = QImage(fileName)

        if image.isNull():
            logger.warn('Invalid image')
            return False

        imageItem = QGraphicsPixmapItem(QPixmap.fromImage(image))
        self.addItem(imageItem)

        return True


class Labeler(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initConsts()
        self.initUI()

    def initConsts(self):
        self.appName = 'Apple Labeler'
        self.resourceDir = 'resources'
        self.iconDir = self.resourceDir + '/icons'

    def initUI(self):
        self.labelScene = LabelScene()
        self.labelView = LabelView()
        self.labelView.setScene(self.labelScene)

        self.statusBar().showMessage('Ready')
        self.createActions()
        self.createMenus()
        self.createToolbar()
        self.createWindow()
        self.setCentralWidget(self.labelView)

    def aboutThisApp(self):
        QMessageBox.about(self, 'About Apple Labeler',
                          '<b>Apple Labeler</b> is an apple labeling tool.')

    def closeEvent(self, event):
        if self.maybeSave():
            event.accept()
        else:
            event.ignore()

    def maybeSave(self):
        # TODO: fix this
        if self.labelScene.modified:
            ret = QMessageBox.warning(self, 'Labeler',
                                      'The image has been labeled.\n'
                                      'Do you want to save your changes?',
                                      QMessageBox.Save |
                                      QMessageBox.Discard | QMessageBox.Cancel)
            if ret == QMessageBox.Save:
                print('save to file')
                return True
            elif ret == QMessageBox.Cancel:
                return False

        return True

    def saveFile(self):
        """
        Save all labels to some kind of file format
        """
        pass

        self.show()

    def createToolbar(self):
        self.toolbar = self.addToolBar('Toolbar')
        self.toolbar.addAction(self.openAct)
        self.toolbar.addAction(self.saveAct)

        self.toolbar.addSeparator()

        self.toolbar.addAction(self.zoomInAct)
        self.toolbar.addAction(self.zoomOutAct)
        # self.toolbar.addAction(self.normalSizeAct)
        # self.toolbar.addAction(self.fitToWindowAct)
        # self.toolbar.addAction(self.zoomAct)

        self.toolbar.addSeparator()

        self.toolbar.addAction(self.selectAct)
        self.toolbar.addAction(self.brushAct)

        self.toolbar.addSeparator()

    def createWindow(self):
        self.setWindowTitle(self.appName)
        self.setWindowIcon(QIcon(self.iconDir + '/apple-512.png'))
        self.centerWindow()

    def getIcon(self, file):
        return QIcon(self.iconDir + '/' + file)

    def createActions(self):
        # File
        # Open
        openIcon = self.getIcon('open-file.png')
        self.openAct = QAction(openIcon, '&Open', self)
        self.openAct.setShortcut('Ctrl+O')
        self.openAct.triggered.connect(self.openFile)

        # Save
        saveIcon = self.getIcon('save-file.png')
        self.saveAct = QAction(saveIcon, '&Save', self)
        self.saveAct.setShortcut('Ctrl+S')
        self.saveAct.setEnabled(False)

        # Exit
        self.exitAct = QAction('&Exit', self)
        self.exitAct.setShortcut('Ctrl+Q')
        self.exitAct.triggered.connect(self.close)

        # View
        # Zoom In
        zoomInIcon = self.getIcon('zoom-in.png')
        self.zoomInAct = QAction(zoomInIcon, 'Zoom &In', self)
        self.zoomInAct.setShortcut('Ctrl++')
        self.zoomInAct.setEnabled(False)

        # Zoom Out
        zoomOutIcon = self.getIcon('zoom-out.png')
        self.zoomOutAct = QAction(zoomOutIcon, 'Zoom &Out', self)
        self.zoomOutAct.setShortcut('Ctrl+-')
        self.zoomOutAct.setEnabled(False)

        # Edit
        # Select
        selectIcon = self.getIcon('select.png')
        self.selectAct = QAction(selectIcon, 'Select', self)
        self.selectAct.setShortcut('Ctrl+E')
        self.selectAct.setCheckable(True)
        self.selectAct.setEnabled(False)

        brushIcon = self.getIcon('brush.png')
        self.brushAct = QAction(brushIcon, 'Brush', self)
        self.brushAct.setShortcut('Ctrl+B')
        self.brushAct.setCheckable(True)
        self.brushAct.setEnabled(False)

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

        # Edit
        editMenu = QMenu('&Edit', self)
        editMenu.addAction(self.selectAct)
        editMenu.addAction(self.brushAct)

        # help
        helpMenu = QMenu('&Help', self)
        helpMenu.addAction(self.aboutAct)
        helpMenu.addAction(self.aboutQtAct)

        # Menu Bar
        menuBar = self.menuBar()
        menuBar.addMenu(fileMenu)
        menuBar.addMenu(viewMenu)
        menuBar.addMenu(editMenu)
        menuBar.addMenu(helpMenu)

    def centerWindow(self):
        frame = self.frameGeometry()
        desktop = QApplication.desktop()
        screen = desktop.screenNumber(desktop.cursor().pos())
        centerPoint = desktop.screenGeometry(screen).center()
        frame.moveCenter(centerPoint)
        self.move(frame.topLeft())
        self.resize(640, 480)

    def openFile(self):
        if self.maybeSave():
            fileName, _ = QFileDialog.getOpenFileName(self, 'Open file',
                                                      QDir.currentPath())

            if fileName:
                if self.labelScene.loadImage(fileName):
                    # Load label file if exists
                    # Parse label file and add items to scene

                    self.selectAct.setEnabled(True)
                    self.brushAct.setEnabled(True)
                else:
                    QMessageBox.information(self, self.appName,
                                            'Cannot load file %s' % fileName)


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    app = QApplication(sys.argv)

    labeler = Labeler()
    labeler.show()

    sys.exit(app.exec_())
