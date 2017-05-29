import logging

from master.fsm import State

logger = logging.getLogger(__name__)

from perception import recognize_objects

class RecognizePhoto(State):
    def run(self):
        photo_urls = [url + '/' for url in self.store.get('/robot/target_photos')]
        locations = [self.store.get(url + '/location') for url in photo_urls]

        # recognize segments
        recognize_objects(self.store, photo_urls, locations)

        self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'rp')
    RecognizePhoto(myname).run()
