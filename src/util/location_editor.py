import logging

import json

from PySide.QtCore import QSize, Qt
from PySide.QtGui import QWidget, QPushButton, QLineEdit, QInputDialog, QApplication, QComboBox, QGridLayout, QPixmap, QLabel, QSizePolicy, QButtonGroup, QRadioButton, QScrollArea

from master.fsm import State

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

def _call(target, *args, **kwargs):
    def _cb(*args2, **kwargs2):
        return target(*args, **kwargs)
    return _cb

class LocationEditor(QScrollArea):
    def __init__(self, store=None):
        super(LocationEditor, self).__init__()

        self.store = store
        if self.store:
            self.locations = {k: v.get('location') for (k, v) in self.store.get(['item'], {}).items()}
        else:
            self.locations = {}

        items = sorted(json.load(open('db/items.json')).values(), key=lambda x: x['name'])
        totes = ['{}_tote'.format(s) for s in json.load(open('db/totes.json')).keys()]
        boxes = ['box{}'.format(b['size_id']) for b in json.load(open('db/boxes.json'))['boxes']]
        bins = json.load(open('db/bins.json')).keys()

        all_locations = sorted(bins) + sorted(boxes) + sorted(totes)

        layout = QGridLayout()

        for (r, item) in enumerate(items):
            picture = QLabel()
            picture.setAlignment(Qt.AlignHCenter)

            pixmap = QPixmap(item.get('thumbnail', ''))
            if not pixmap.isNull():
                picture.setPixmap(pixmap)
            layout.addWidget(picture, r, 0)

            label = QLabel(item.get('display_name', item['name']))
            #label.setStyleSheet('font-weight: bold;')
            layout.addWidget(label, r, 1)

            group = QButtonGroup(self)
            for (c, location) in enumerate(all_locations):
                button = QPushButton(location)
                button.setCheckable(True)
                button.clicked.connect(_call(self.set_location, item['name'], location))
                button.enterEvent = _call(self.highlight, label)
                #button.leaveEvent = _call(self.highlight, None)

                button.setChecked(self.locations.get(item['name']) == location)

                group.addButton(button)
                layout.addWidget(button, r, 2 + c)


        widget = QWidget()
        widget.setLayout(layout)

        widget.leaveEvent = _call(self.highlight, None)

        self.setWidget(widget)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setMinimumWidth(widget.sizeHint().width() + self.verticalScrollBar().sizeHint().width())

        self.setWindowTitle('Location Editor')
        self.setStyleSheet('QPushButton:checked { font-weight: bold; color: blue;}')

        self.last_highlight = None

    def highlight(self, label):
        if self.last_highlight:
            self.last_highlight.setStyleSheet('')

        if label:
            label.setStyleSheet('font-weight: bold; color: blue;')

        self.last_highlight = label

    def set_location(self, item, location):
        self.locations[item] = location

        if self.store:
            self.store.put(['item', item, 'location'], location)
            logger.info('set "{}" to "{}"'.format(item, location))


if __name__ == '__main__':
    app = QApplication([])
    # import argparse
    # parser = argparse.ArgumentParser()
    # parser.add_argument('name', nargs='?')
    # args = parser.parse_args()
    # myname = (args.name or 'csi')
    # csi = CheckSelectItem(myname)
    # csi.run()

    from pensive.client import PensiveClient

    window = LocationEditor(store=PensiveClient().default())
    window.show()

    app.exec_()
