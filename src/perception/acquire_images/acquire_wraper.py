import json
import subprocess
import sys
import fcntl
import os
if __name__ == "__main__":
    #read in NEW item config. All 40 items 20 new
    with open('../../../db/items.json') as data_file:
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
    'toilet_brush']
    # 'white_facecloth'
    # 'windex']


    #make a directory for all of the images
    try:
        os.mkdir("images")
    except:
        print("Directory images already exists")

    for item in objectnames:
        if not item in old_objects:
            cnt = 0
            while(True):
                #go until the user says stop
                print('Taking pictures of {}, okay? ([y]/n)'.format(item))
                inp = sys.stdin.readline()
                if inp == 'y\n' or inp == '\n':
                    #make new directory
                    dirname = "images/" + item + str(cnt)
                    try:
                        os.mkdir(dirname)
                        
                    except:
                        print("could not make directory {}".format(dirname))

                    with open(dirname + "/" + 'lockfile', 'w') as fp:
                        #lock the file
                        fcntl.lockf(fp.fileno(), fcntl.LOCK_EX)
                        #start subprocess
                        try:
                            subprocess.check_output(["./acquire_images", "{}{}".format("images/"+item, str(cnt))])
                        except:
                            print("Acquire images returned non-zero status")
                        
                        #unlock
                        fcntl.lockf(fp.fileno(), fcntl.LOCK_UN)
                    cnt += 1
                else:
                    break
            
