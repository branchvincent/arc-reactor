import numpy as np
import math
import sys
import perception.deepLearning as dl
from pensive.client import PensiveClient
from pensive.coders import register_numpy
from time import sleep
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
        #load in the items.json file
        logger.critical('using hard-coded items file')
        with open('db/items.json') as data_file:
            jsonnames = json.load(data_file)
        for key,value in jsonnames.items():
            names.append((key, value['mass']))
        names.sort(key=lambda tup: tup[0])
        self.object_names = names

        self.deep_learning_recognizer = dl.DeepLearningRecognizer(network_name,len(names))

        #given a location returns a list of items in the location
        self.items_in_location = {}


    def poll_database(self):
        '''
        Polls the database at a specific URL until a flag is set.
        Once the flag is set run inference and return to polling
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
                    use_weight = self.store.get('/object_recognition/use_weight')
                    self.infer_objects(list_of_urls, list_of_locations, use_weight)
                    logger.info("Inference complete")
                    self.store.put('/object_recognition/done', 1)
                else:
                    sleep(0.1)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            logger.exception('Inference failed! {}'.format(e))
            self.store.put('object_recognition/error', e)
            raise


    def infer_objects(self, list_of_urls, list_of_locations, list_of_use_weight):
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
                self.store.put("object_recognition/error", error_string)
                self.store.put(url + 'detections', [])
                return

            #store all deep learning confidences for each image
            confidences = self.infer_objects_deep(dl_images)
            list_of_list_of_confidences_deep = self.filter_confidences(confidences, list_of_locations[i])
            self.store.put(url + 'detections', list_of_list_of_confidences_deep)

            #store all confidences for weight
            #only if there is a weight
            if list_of_use_weight[i]:

                logger.info("Inferring object based on weight")
                weight = abs(self.store.get('/scales/change'))
                stdev = self.store.get('/system/scales/stdev',0.005)

                if weight is None or stdev is None:
                    error_string = "Weight or stdev was none"
                    logger.error(error_string)
                    self.store.put("object_recognition/error", error_string)
                    return

                confidences = self.infer_objects_weight(weight, stdev)
                list_of_list_of_confidences_weight = self.filter_confidences(confidences, list_of_locations[i])
                self.store.put(url + 'detections_weight', list_of_list_of_confidences_weight)


                #check to see if there were too many images at the inspection station
                if len(list_of_list_of_confidences_deep) > 1:
                    error_string = "Too many images for inspection station. Expecting 1. Got {}. Using first image".format(len(list_of_list_of_confidences_deep))
                    logger.warning(error_string)
                    self.store.put('object_recognition/error',error_string)

                #combine the confidences and write that out to detections
                #combine the first list of confidences from dl with weights
                combined_dict = {}
                for key,value in list_of_list_of_confidences_deep[0].items():
                    combined_dict[key] = value*list_of_list_of_confidences_weight[0][key]

                self.store.put(url + 'detections_combined', [combined_dict])

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


    def infer_objects_weight(self, measured_weight, stdev):
        '''
        assumes that all weights and stdev are in kg
        returns list of probabilities that the item is the same one as
        the item with "measured_weight"
        '''
        list_of_weight_prob = []
        for w in self.object_names:
            prob = scipy.stats.norm(w[1],stdev).pdf(measured_weight)/1000
            list_of_weight_prob.append(prob)

        return np.array(list_of_weight_prob).reshape(1,40)

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
                res.append((self.object_names[i][0], float(list_of_conf[i])))
        else:
        #set the confidence of items that are not valid_items to zero
            for i in range(len(list_of_conf)):
                if self.object_names[i][0] in valid_items:
                    res.append((self.object_names[i][0], float(list_of_conf[i])))
                else:
                    res.append((self.object_names[i][0], 0))

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
                logger.warning("Unknown itme location {}".format(location))


if __name__ == '__main__':
    o = ObjectRecognition('db/resnet_finetuned_06132017_combined.pkl')
    o.poll_database()