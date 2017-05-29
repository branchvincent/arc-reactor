import logging
from master.fsm import State
from hardware.dymo.scale import Scale as DymoScale
from hardware.atron.scale import Scale as AtronScale
logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class ReadScales(State):
    def run(self):
        # Get scales
        scales = []
        for key, val in self.store.get('system/scales', {}).iteritems():
            if key.startswith('dymo'):
                scales.append(DymoScale(port=val))
            elif key.startswith('atron'):
                scales.append(AtronScale(port=val))
                # scales.append(AtronScale(serial=scale[:-1], port=scale[-1]))

        # Read total weight
        weight = 0
        for scale in scales:
            weight += scale.read()

        # Update database
        self.store.put('scales/weight', weight)

        self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or rs')
    ReadScales(myname).run()
