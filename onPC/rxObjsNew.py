import sys
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np

app = QtGui.QApplication(sys.argv)
mw = QtGui.QMainWindow()
mw.resize(800, 800)
view = pg.GraphicsLayoutWidget()
mw.setCentralWidget(view)
mw.setWindowTitle('pyqtgraph example: ScatterPlot')
w1 = view.addPlot()



x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
y = [10, 8, 6, 4, 2, 20, 18, 16, 14, 12]

# Create seed for the random
time = QtCore.QTime.currentTime()
QtCore.qsrand(time.msec())

for i in range(len(x)):
    s = pg.ScatterPlotItem([x[i]], [y[i]], size=10, pen=pg.mkPen(None))  # brush=pg.mkBrush(255, 255, 255, 120))
    s.setBrush(QtGui.QBrush(QtGui.QColor(QtCore.qrand() % 256, QtCore.qrand() % 256, QtCore.qrand() % 256)))
    w1.addItem(s)
mw.show()
sys.exit(QtGui.QApplication.exec_())