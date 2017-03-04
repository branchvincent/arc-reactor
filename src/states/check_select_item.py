import logging
import sys
from master.fsm import State
from PySide.QtGui import QWidget, QPushButton, QLineEdit, QInputDialog, QApplication, QComboBox

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class CheckSelectItem(State):
    def run(self):
        app = QApplication.instance()
        wd = WidgetDialog(self.store)
        #wd.show()
        #sys.exit(app.exec_())
        ret = app.exec_()
        #sys.exit(ret)

class WidgetDialog(QWidget):
    def __init__(self, store):
        self.store = store
        super(WidgetDialog, self).__init__()
        self.initUI()
    
    def initUI(self):
        self.btn = QPushButton('Change item', self)
        self.btn.move(10, 50)
        self.btn.clicked.connect(self.getItem)

        self.le = QLineEdit(self)
        self.le.move(10, 20)
        self.le.setFixedSize(300, 30)
        self.le.setText(self.store.get('/robot/selected_item'))

        self.btnClose = QPushButton('Accept item', self)
        self.btnClose.move(130, 50)
        self.btnClose.clicked.connect(self.closeMe)

        self.setGeometry(300, 300, 350, 150)
        self.show()

    def closeMe(self):
        QApplication.quit()

    def getItem(self):
        items = []
        self.itemList = self.store.get('/item/')
        for k, n in self.itemList.items():
            items.append(self.itemList[k]['name'])

        item, ok = QInputDialog.getItem(self, 'Select item', 'Available items:', items, 0, False)
        if ok and item:
            self.le.setText(item)

        self.store.put('/robot/selected_item', item)


##########################
def main():
    csi = CheckSelectItem('csi')
    csi.run()

if __name__ == '__main__':
    main()

