import logging

from master.fsm import State

from hardware.dymo.scale import Scale as DymoScale

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class ReadScalesStow(State):
    """
    Input:
        - /system/scales: dictionary of scale names
        - /scales/last_weight (optional): last recorded weight
    Output:
        - /scales/weight: current weight
        - /scales/change: difference between current weight and last weight
        - /failure/read_scales: failure string
    Failure Cases:
        - scale_error: error reading from scale (TODO: classify errors)
    Dependencies:
        - None
    """

    def run(self):
        self.store.put('scales/last_weight', self.store.get('scales/weight', 0))
        # Get scales
        scales = []
        for key, val in self.store.get('system/scales', {}).iteritems():
            if key.startswith('dymo'):
                scales.append(DymoScale(port=val))
            elif key.startswith('atron'):
                pass

        # Read total weight
        weights = [scale.read() for scale in scales]
        total_weight = sum(weights)

        weight_change = total_weight - self.store.get('scales/last_weight', 0)

        logger.info('weight {} = {:.3f} kg'.format(weights, total_weight))
        logger.info('change = {:.3f} kg'.format(weight_change))

        # Update database
        self.store.put('scales/weight', total_weight)
        self.store.put('scales/change', weight_change)
        #TODO fail if scales hardware produces an error
        # self.store.put('failure/read_scales', 'scale_error')
        self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'rss')
    ReadScalesStow(myname).run()
