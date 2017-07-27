import json
import subprocess
import sys
import fcntl
import os
import sys
import cv2
from pensive.client import PensiveClient
from pensive.coders import register_numpy



if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Requires a json file name")
        return     
    json_file_name = sys.argv[1]
    #get a database
    store = PensiveClient().default()
    register_numpy()

    #read in NEW item config. This has all of the items we care about, might not be 40
    with open(json_file_name) as data_file:
            jsonnames = json.load(data_file)
    names=[]
    for key,value in jsonnames.items():
        names.append(key)
    names.sort()
    objectnames = [x.lower() for x in names]
    
    #remove objects from the old list
    old_objects = ['avery_binder',
    'balloons',
    'band_aid_tape',
    'bath_sponge',
    'black_fashion_gloves',
    'burts_bees_baby_wipes',
    'colgate_toothbrush_4pk',
    'composition_book',
    'crayons',
    'duct_tape',
    'epsom_salts',
    'expo_eraser',
    'fiskars_scissors',
    'flashlight',
    'glue_sticks',
    'hand_weight',
    'hanes_socks',
    'hinged_ruled_index_cards',
    'ice_cube_tray',
    'irish_spring_soap',
    'laugh_out_loud_jokes',
    'marbles',
    'measuring_spoons',
    'mesh_cup',
    'mouse_traps',
    'pie_plates',
    'plastic_wine_glass',
    'poland_spring_water',
    'reynolds_wrap',
    'robots_dvd',
    'robots_everywhere',
    'scotch_sponges',
    'speed_stick',
    'table_cloth',
    'tennis_ball_container',
    'ticonderoga_pencils',
    'tissue_box',
    'toilet_brush',
    'white_facecloth',
    'windex']


    #make a directory for all of the images
    try:
        os.mkdir("images")
    except:
        print("Directory images already exists")

    for item in objectnames:
        if not item in old_objects:
            #make new directory
            dirname = "images/" + item
            try:
                os.mkdir(dirname)
            except:
                print("could not make directory {}".format(dirname))

            while(True):
                cnt = 0
                #place all new images in this directory, they will already be segmented
                #go until the user says stop
                print('Taking pictures of {}, okay? ([y]/n)'.format(item))
                inp = sys.stdin.readline()
                if inp == 'y\n' or inp == '\n':
                    #start subprocess
                    try:
                        subprocess.check_output(["./reactor shell states.capture_photo_inspect"])
                    except:
                        print("Capture photo inspect didn't work")


                    try:
                        subprocess.check_output(["./reactor shell segment_photo"])
                    except:
                        print("Segment photo didn't work")

                            
                    #save the segmented images
                    mask_bot = store.get("/photos/inspect/inspect_below/labeled_image")
                    mask_side = store.get("/photos/inspect/inspect_side/labeled_image")

                    image_bot = store.get("/photos/inspect/inspect_below/full_color")
                    image_side = store.get("/photos/inspect/inspect_side/full_color")

                    seg_bot = image_bot.copy()
                    seg_bot[:,:,0] = image_bot[:,:,0]*mask_bot
                    seg_bot[:,:,1] = image_bot[:,:,1]*mask_bot
                    seg_bot[:,:,2] = image_bot[:,:,2]*mask_bot

                    seg_side = image_side.copy()
                    seg_side[:,:,0] = image_side[:,:,0]*mask_side
                    seg_side[:,:,1] = image_side[:,:,1]*mask_side
                    seg_side[:,:,2] = image_side[:,:,2]*mask_side

                    #save it out
                    cv2.imwrite("images/{}/side{}.png".format(item,cnt),seg_side)
                    cv2.imwrite("images/{}/bot{}.png".format(item,cnt),seg_bot)

                    cnt += 1
                else:
                    break
            
            #done taking pictures time to weigh
            try:
                subprocess.check_output(["./reactor shell states.read_scales_pick"])
                subprocess.check_output(["./reactor shell states.read_scales_pick"])
                subprocess.check_output(["./reactor shell states.read_scales_pick"])
            except:
                print("Something went wrong with reading the scales")

            print("Put the {} on the scales".format(item))
            inp = sys.stdin.readline()
            try:
                subprocess.check_output(["./reactor shell states.read_scales_pick"])
            except:
                print("Something went wrong with reading the scales")


            #get the weight
            weight = abs(store.get("/scales/change"))
            print("Weight was {}".format(abs(weight)))

            #modify the dictonary
            jsonnames[item]['mass'] = weight
            print("Weight updated. Take {} off the scales".format(item))

    #write out the josn file
    with open(json_file_name, 'w') as outfile:
        json.dump(jsonnames, outfile)