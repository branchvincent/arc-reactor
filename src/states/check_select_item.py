import logging

from PySide.QtCore import QSize, Qt
from PySide.QtGui import QWidget, QPushButton, QLineEdit, QInputDialog, QApplication, QComboBox, QGridLayout, QIcon, QLabel, QSizePolicy

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

        items = self.store.get('/item')

        layout = QGridLayout()
        self.setLayout(layout)

        self.item_buttons = {}

        width = 5

        label = QLabel('Selected Item')
        label.setAlignment(Qt.AlignHCenter)
        label.setStyleSheet('font-weight: bold;')
        layout.addWidget(label, 0, 0, 1, width)

        for (i, item) in enumerate(sorted(items)):
            parts = items[item]['display_name'].split(' ') + ['({})'.format(items[item]['location'])]
            lines = ['']
            for part in parts:
                if len(lines[-1]) > 10:
                    lines.append('')

                lines[-1] += ' ' + part

            button = QPushButton('\n'.join([line.strip() for line in lines]))
            icon = QIcon(items[item].get('thumbnail', ''))
            if not icon.isNull():
                button.setIcon(icon)
                button.setIconSize(QSize(64, 64))
            button.setCheckable(True)

            button.clicked.connect(_call(self.select_item, item))

            self.item_buttons[item] = button
            layout.addWidget(button, i // width + 1, i % width)

        self.select_item(self.store.get('/robot/selected_item'))

        self.box_buttons = {}

        label = QLabel('Target Box')
        label.setAlignment(Qt.AlignHCenter)
        label.setStyleSheet('font-weight: bold;')
        layout.addWidget(label, 0, width + 1)

        for (i, box) in enumerate(sorted(self.store.get('/box'))):
            button = QPushButton(box)
            button.setCheckable(True)
            button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            button.clicked.connect(_call(self.select_box, box))

            self.box_buttons[box] = button
            layout.addWidget(button, i + 1, width + 1)

        self.select_box(self.store.get('/robot/target_box'))

        self.bin_buttons = {}

        label = QLabel('Target Bin')
        label.setAlignment(Qt.AlignHCenter)
        label.setStyleSheet('font-weight: bold;')
        layout.addWidget(label, 0, width + 2)

        for (i, bin) in enumerate(sorted(self.store.get('/shelf/bin'))):
            button = QPushButton(bin)
            button.setCheckable(True)
            button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            button.clicked.connect(_call(self.select_bin, bin))

            self.bin_buttons[bin] = button
            layout.addWidget(button, i + 1, width + 2)

        self.select_bin(self.store.get('/robot/target_bin'))

    def select_item(self, selected_item):
        for (item, button) in self.item_buttons.items():
            if item == selected_item:
                button.setStyleSheet('color: blue; font-weight: bold;')
                button.setChecked(True)
            else:
                button.setStyleSheet('')
                button.setChecked(False)

        self.store.put('/robot/selected_item', selected_item)
        logger.info('selected item: "{}"'.format(selected_item))

    def select_box(self, target_box):
        for (box, button) in self.box_buttons.items():
            if box == target_box:
                button.setStyleSheet('color: blue; font-weight: bold;')
                button.setChecked(True)
            else:
                button.setStyleSheet('')
                button.setChecked(False)

        self.store.put('/robot/target_box', target_box)
        logger.info('target box: "{}"'.format(target_box))

    def select_bin(self, target_bin):
        for (bin, button) in self.bin_buttons.items():
            if bin == target_bin:
                button.setStyleSheet('color: blue; font-weight: bold;')
                button.setChecked(True)
            else:
                button.setStyleSheet('')
                button.setChecked(False)

        self.store.put('/robot/target_bin', target_bin)
        logger.info('target bin: "{}"'.format(target_bin))

if __name__ == '__main__':
    app = QApplication([])

    csi = CheckSelectItem('csi')
    csi.run()

    app.exec_()
