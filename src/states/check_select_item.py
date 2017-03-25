import logging

from PySide.QtCore import QSize
from PySide.QtGui import QWidget, QPushButton, QLineEdit, QInputDialog, QApplication, QComboBox, QGridLayout, QIcon

from master.fsm import State

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

def _call(target, *args, **kwargs):
    def _cb():
        return target(*args, **kwargs)
    return _cb

class CheckSelectItem(State):
    def run(self):
        window = ItemSelector(self.store)
        window.show()

class ItemSelector(QWidget):
    def __init__(self, store):
        super(ItemSelector, self).__init__()

        self.setWindowTitle('Select Item Checkpoint')

        self.store = store

        items = self.store.get('/item').keys()

        layout = QGridLayout()
        self.setLayout(layout)

        self.buttons = {}

        width = 5
        for (i, item) in enumerate(sorted(items)):
            parts = item.replace('_', ' ').title().split()
            lines = ['']
            for part in parts:
                if len(lines[-1]) > 10:
                    lines.append('')

                lines[-1] += ' ' + part

            button = QPushButton('\n'.join([line.strip() for line in lines]))
            button.setIcon(QIcon('data/objects/apc2017/{0}/thumb.png'.format(item)))
            button.setIconSize(QSize(64, 64))
            button.setCheckable(True)

            button.clicked.connect(_call(self.select, item))

            self.buttons[item] = button
            layout.addWidget(button, i // width, i % width)

        self.select(self.store.get('/robot/selected_item'))

    def select(self, selected_item):
        for (item, button) in self.buttons.items():
            if item == selected_item:
                button.setStyleSheet('color: blue; font-weight: bold;')
                button.setChecked(True)
            else:
                button.setStyleSheet('')
                button.setChecked(False)

        self.store.put('/robot/selected_item', selected_item)
        logger.info('selected item: "{}"'.format(selected_item))

if __name__ == '__main__':
    app = QApplication([])

    csi = CheckSelectItem('csi')
    csi.run()

    app.exec_()
