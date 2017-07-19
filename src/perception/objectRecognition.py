import numpy as np
import math
import perception.deepLearning as dl
from pensive.client import PensiveClient
from pensive.coders import register_numpy
from time import sleep
import pickle
import json
# configure the root logger to accept all records
import logging
logger = logging.getLogger(__name__)
import scipy.stats

class ObjectRecognition:

    def __init__(self, network_name):
        #try to connect to the database
        self.store = None
        try:
            db_client = PensiveClient()
            self.store = db_client.default()
            register_numpy()
        except:
            raise RuntimeError('Could not connect to the database. Cannot function')

        names = []

        #load in the dictionary that is index in deep learning output -> item name
        try:
            self.deep_learning_index = pickle.load(open('db/deep_learning_index.pkl', 'rb'))
        except:
            errstr = "Cannot open the item name to deep learning index dictionary."
            logger.critical(errstr)
            raise RuntimeError(errstr)

        #load in the items.json file
        with open('db/items.json') as data_file:
            jsonnames = json.load(data_file)
        for key,value in jsonnames.items():
            names.append(key)
        names.sort()
        self.object_names = names

        self.deep_learning_recognizer = dl.DeepLearningRecognizer(network_name,len(names))

        #given a location returns a list of items in the location
        self.items_in_location = {}


    def poll_database(self):
        '''
        Polls the database at a specific URL until a flag is set.
        Once the flag is set run inference and return to polling
        Looking for URLs where deep learning images are
        Locations are what physical location the item were taken from (binA, tote)
        '''
        try:
            while True:
                should_run = self.store.get('/object_recognition/run')
                if should_run:
                    logger.info("Starting inference")
                    self.store.put('/object_recognition/run',0)
                    #get the arguments
                    list_of_locations = self.store.get('/object_recognition/locations')
                    list_of_urls = self.store.get('/object_recognition/urls')
                    self.infer_objects(list_of_urls, list_of_locations)
                    logger.info("Inference complete")
                    self.store.put('/object_recognition/done', 1)

                #check for multiple image running
                should_run_multi = self.store.get('/object_recognition/multi_run')
                if should_run_multi:

                    logger.info("Starting a multi inference")
                    self.store.put('/object_recognition/multi_run',0)
                    #looking for a list of lists of urls
                    #each list will have have urls for multiple images
                    list_of_urls = self.store.get('/object_recognition/multi_urls')

                    #need a list of lists of indicies for each of the DL images
                    #this gives the segment in the labeled image that we will use to determine
                    #the size of the segment
                    list_of_indices = self.store.get('/object_recognition/multi_indices')

                    #where was this item picked from?
                    location = self.store.get('/object_recognition/multi_location')

                    self.multi_image_inference(list_of_urls, list_of_indices, location, weight)
                    self.store.put('/object_recognition/multi_done', 1)
                    logger.info("Multi inference done")

                else:
                    sleep(0.1)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            logger.exception('Inference failed! {}'.format(e))
            self.store.put('object_recognition/error', 'Inference failed! {}'.format(e))
            raise


    def infer_objects(self, list_of_urls, list_of_locations):
        '''
        Given a list of urls and a list of locations infer what each image is at the URL
        and post the entire list of confidences. Set confidence of object not at location to zero
        '''
        error_string = None
        if len(list_of_urls) != len(list_of_locations):
            error_string = "Length mismatch of url list and location list. Not guessing objects"
            logger.warning(error_string)
            self.store.put("object_recognition/error", error_string)
            return
        elif len(list_of_urls) == 0 or list_of_urls is None:
            error_string = "No URLs were passed. Need at least one. Not guessing objects"
            logger.warning(error_string)
            self.store.put("object_recognition/error", error_string)
            return
        elif len(list_of_locations) == 0 or list_of_locations is None:
            error_string = "No locations were passed. Need at least one. Not guessing objects"
            logger.warning(error_string)
            self.store.put("object_recognition/error", error_string)
            return

        #find which items are at each location
        self.update_item_locations()

        for i, url in enumerate(list_of_urls):
            #get the list of images at this URL
            dl_images = self.store.get(url + 'DL_images')
            if dl_images is None:
                error_string = "No deep learning images were found at the URL {}".format(url)
                logger.error(error_string)
                self.store.put("object_recognition/error", error_string)
                return
            if len(dl_images) == 0:
                #empty list
                error_string = "List of images was empty. Sending out empty list. GARBAGE IN, GARBAGE OUT"
                logger.warning(error_string)
                self.store.put("object_recognition/error", None)
                self.store.put(url + 'detections', [])
                return

            #store all deep learning confidences for each image
            confidences = self.infer_objects_deep(dl_images)
            list_of_list_of_confidences_deep = self.filter_confidences(confidences, list_of_locations[i])
            self.store.put(url + 'detections', list_of_list_of_confidences_deep)
            

        self.store.put('object_recognition/error',error_string)

    def infer_objects_deep(self, list_of_dl_images):
        '''
        Get confidence level of all items for all images using deep learning
        '''
        #guess all objects at once
        im = np.zeros((224,224,3,len(list_of_dl_images)))
        for num,img in enumerate(list_of_dl_images):
            im[:,:,:,num] = img

        #infer
        logger.info("Runing {} images through network".format(len(list_of_dl_images)))
        confidences = self.deep_learning_recognizer.guessObject(im)
        return confidences



    def multi_image_inference(self, list_of_urls, list_of_indices, location, weight):
        error_string = None
        #check to see if the inputs are valid
        if list_of_indices is None or list_of_urls is None or location is None or weight is None:
            error_string = "Inputs to multi image inference can't be none"
            logger.error(error_string)
            self.store.put("object_recognition/error", error_string)
            return
        if len(list_of_indices) != len(list_of_urls):
            error_string = "Length of lists of indices != length of lists of urls"
            logger.error(error_string)
            self.store.put("object_recognition/error", error_string)
            return

        #find which items are at each location
        self.update_item_locations()

        list_of_dl_images = []
        list_of_pixel_counts = []

        for url, index in zip(list_of_urls, list_of_indices):
            #get the list of images at this URL
            dl_images = self.store.get(url + 'DL_images')
            if dl_images is None:
                error_string = "No deep learning images were found at the URL {}".format(url)
                logger.error(error_string)
                self.store.put("object_recognition/error", error_string)
                return

            #get the dl image at index
            dl_image = dl_images[index-1]

            #get the number of pixels in the labeled image at this index
            labled_image = self.store.get(url + 'labeled_image')
            if labled_image is None:
                error_string = "No labeled image found at the URL {}".format(url)
                logger.error(error_string)
                self.store.put("object_recognition/error", error_string)
                return

            num_pixels = len(np.where(labled_image==index)[0])
            list_of_dl_images.append(dl_image)
            list_of_pixel_counts.append(num_pixels)

        #array NX40 where N is number of images
        dl_confidences = self.infer_objects_deep(list_of_dl_images)

        #narrow down confidences based on the location, output is list of lists
        dl_confidences_filtered = self.filter_confidences(dl_confidences, location)

        max_confidence = -1000
        guessed_item = None
        list_of_combined_confidences = []
        for i in range(len(list_of_urls)):
            combined_dict = {}
            for key,value in dl_confidences_filtered[i].items():
                    val = value*list_of_pixel_counts[i]
                    combined_dict[key] = val
                    if val > max_confidence:
                        max_confidence = val
                        guessed_item = key
            list_of_combined_confidences.append(combined_dict)

        #write it out to the database
        self.store.put('/object_recognition/multi_results',list_of_combined_confidences)
        self.store.put('/object_recognition/multi_highest_confidence', max_confidence)
        self.store.put('/object_recognition/multi_best_guess', guessed_item)
        self.store.put("/object_recognition/error", error_string)

    def filter_confidences(self, confidences, location):
        '''
        Filter the list of list of confidences by assigning zero
        to items that are not in the location. If no location is provided,
        or is incorrect don't filter
        '''
        list_of_list_of_confidences_out = []

        for list_of_conf in confidences:
            #see if a valid location was passed in
            try:
                items_at_location = self.items_in_location[location]
            except KeyError:
                logger.warning("Bad location name {}. Sending all confidences".format(location))
                items_at_location = None

            item_name_confidence = self.make_name_confidence_list(list_of_conf, items_at_location)
            list_of_list_of_confidences_out.append(dict(item_name_confidence))

        return list_of_list_of_confidences_out

    def make_name_confidence_list(self, list_of_conf, valid_items):
        res = []

        if valid_items is None:
            for i in range(len(list_of_conf)):
                res.append((self.deep_learning_index[i], float(list_of_conf[i])))
        else:
        #set the confidence of items that are not valid_items to zero
            for i in range(len(list_of_conf)):
                item = self.deep_learning_index[i]
                if item in valid_items:
                    res.append((item, float(list_of_conf[i])))
                else:
                    res.append((item, 0))

        return res

    def update_item_locations(self):

        items = self.store.get('/item/')
        if items is None:
            logger.warn("Item dictionary returned was None. Can't update item locations")
            return

        #reset the dictionary
        self.items_in_location['binA'] = []
        self.items_in_location['binB'] = []
        self.items_in_location['binC'] = []
        self.items_in_location['amnesty_tote'] = []
        self.items_in_location['stow_tote'] = []


        for key, value in items.items():
            #where is this item? binA/B/C, tote?
            location = value['location']
            if 'bin' in location:
                if location[3] == 'A':
                    self.items_in_location['binA'].append(value['name'])
                elif location[3] == 'B':
                    self.items_in_location['binB'].append(value['name'])
                elif location[3] == 'C':
                    self.items_in_location['binC'].append(value['name'])
                else:
                    logger.warning("Invalid bin ({}) for item {}.".format(location, value['name']))
            elif 'tote' in location:
                #tote cam
                if 'amnesty' in location:
                    self.items_in_location['amnesty_tote'].append(value['name'])
                elif 'stow' in location:
                    self.items_in_location['stow_tote'].append(value['name'])
            else:
                logger.warning("Unknown item location {}".format(location))


if __name__ == '__main__':
    o = ObjectRecognition('db/resnet_finetuned_06132017_combined.pkl')
    o.poll_database()