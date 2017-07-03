import csv

import logging

from pensive.client import PensiveClient

logger = logging.getLogger(__name__)

def run(path, name_column, mass_column, headers=1, store=None):
    store = store or PensiveClient().default()

    with open(path, 'rb') as f:
        reader = csv.reader(f)
        for (i, row) in enumerate(reader):
            if i < headers:
                continue

            name = row[name_column]
            mass = float(row[mass_column])

            if store.get(['item', name]):
                logger.info('{} -> {:.3f} kg'.format(name, mass))
                store.put(['item', name, 'mass'], mass)


if __name__ == '__main__':
    import sys
    run(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]))

