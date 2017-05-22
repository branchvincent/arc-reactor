import numpy as np
import math
import sys
sys.path.append('..')
import deepLearning as dl
from pensive.client import PensiveClient
from pensive.coders import register_numpy
from time import sleep
import logging
logger = logging.getLogger(__name__)

class ObjectRecognition:

     def __init__(self, network_name):
        #try to connect to the database
        self.store = None
        try:
            db_client = PensiveClient(host='http://10.10.1.60:8888')
            self.store = db_client.default()
            register_numpy()
        except:
            logger.error("Could not connect to the database on {}. Cannot function".format('10.10.1.60:8888'))
            return
        
        self.deep_learning_recognizer = dl.DeepLearningRecognizer(network_name,40)
    

        #load in the items.json file
        with open('../../db/items.json') as data_file:    
            jsonnames = json.load(data_file)
        for key,value in jsonnames.items():
            names.append(key)
        names.sort()
        self.object_names = names


        #given a location returns a list of items in the location
        self.items_in_location = {}


    '''
    Polls the database at a specific URL until a flag is set.
    Once the flag is set run inference and return to polling
    '''
    def poll_database(self):

        while True:
            should_run = self.store.get('/object_recognition/run')
            if should_run == 1:
                #get the arguments
                list_of_locations = self.store.get('/object_recognition/locations')
                list_of_urls = self.store.get('/object_recognition/urls')
                infer_objects(list_of_urls, list_of_locations)
            else:
                sleep(0.1)

    '''
    Given a list of urls and a list of locations infer what each image is at the URL
    and post the entire list of confidences. Set confidence of object not at location to zero
    '''
    def infer_objects(self, list_of_urls, list_of_locations):
        
        if len(list_of_urls) != len(list_of_locations):
            logger.warning("Length mismatch of url list and location list. Not guessing objects")
            return
        elif len(list_of_urls) == 0 or list_of_urls is None:
            logger.warning("No URLs were passed. Need at least one. Not guessing objects")
            return
        elif len(list_of_locations) == 0 or list_of_locations is None:
            logger.warning("No locations were passed. Need at least one. Not guessing objects")
            return
        
        #find which items are at each location
        self.update_item_locations()

        for i, url in enumerate(list_of_urls):
            #get the list of images at this URL
            dl_images = store.get(url + 'DL_images')
            if dl_images is None:
                logger.error("No deep learning images were found at the URL {}".format(url))
                continue
        
            #guess all objects at once
            im = np.zeros((224,224,3,len(dl_images)))
            for num,img in enumerate(dl_images):
                im[:,:,:,num] = img

            #infer
            confidences = self.deep_learning_recognizer.guessObject(im)

            #store all confidences for each image
            for list_of_conf in confidences:
                #see if a valid location was passed in
                try:
                    items_at_location = self.items_in_location[list_of_locations[i]]
                except:
                    logger.warning("Bad location name {}. Sending all confidences".format(list_of_locations[i]))
                    items_at_location = None
                
                item_name_confidence = self.make_name_confidence_list(list_of_conf, items_at_location)
                store.put(url + 'detections', item_name_confidence)
    
    def make_name_confidence_list(self, list_of_conf, valid_items):
        res = []

        if valid_items is None:
            for i in range(len(list_of_conf)):
                res.append((self.object_names[i], list_of_conf[i]))
        else:
        #set the confidence of items that are not valid_items to zero
            for i in range(len(list_of_conf)):
                if self.object_names[i] in valid_items:
                    res.append((self.object_names[i], list_of_conf[i]))
                else:
                    res.append((self.object_names[i], 0))

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
                if location[3] == 'A
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
