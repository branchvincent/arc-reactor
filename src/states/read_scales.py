import logging
from master.fsm import State
from hardware.dymo.scale import Scale as DymoScale
from hardware.atron.scale import AtronScale
logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class ReadScales(State):
    def run(self):

        self.store.put('scales/last_weight', self.store.get('scales/weight', 0))
        # Get scales
        scales = []
        for key, val in self.store.get('system/scales', {}).iteritems():
            if key.startswith('dymo'):
                #scales.append(DymoScale(port=val))
                pass
            elif key.startswith('atron'):
                scales.append(AtronScale(key, self.store))

        # Read total weight
        weights = [scale.read() for scale in scales]
        total_weight = sum(weights)

        logger.info('weight {} = {:.3f} kg'.format(weights, total_weight))

        # Update database
        self.store.put('scales/weight', total_weight)
        self.store.put('scales/change', total_weight-self.store.get('scales/last_weight', 0))
        #TODO fail if scales hardware produces an error
        self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'rs')
    ReadScales(myname).run()
