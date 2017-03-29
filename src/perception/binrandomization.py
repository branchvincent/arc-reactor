import time
import sys
import random
import json
import copy

def output_random_bin(times, output):
    item_list = ['Avery_Binder', 'Balloons', 'Band_Aid_Tape', 'Bath_Sponge', 'Black_Fashion_Gloves',
    'Burts_Bees_Baby_Wipes', 'Colgate_Toothbrush_4PK', 'Composition_Book', 'Crayons',
    'Duct_Tape', 'Epsom_Salts', 'Expo_Eraser', 'Fiskars_Scissors', 'Flashlight', 'Glue_Sticks',
    'Hand_Weight', 'Hanes_Socks', 'Hinged_Ruled_Index_Cards', 'Ice_Cube_Tray', 'Irish_Spring_Soap',
    'Laugh_Out_Loud_Jokes', 'Marbles', 'Measuring_Spoons', 'Mesh_Cup', 'Mouse_Traps',
    'Pie_Plates', 'Plastic_Wine_Glass', 'Poland_Spring_Water', 'Reynolds_Wrap', 'Robots_DVD',
    'Robots_Everywhere', 'Scotch_Sponges', 'Speed_Stick', 'Table_Cloth', 'Tennis_Ball_Container',
    'Ticonderoga_Pencils', 'Tissue_Box', 'Toilet_Brush', 'White_Facecloth', 'Windex']

   
    random.seed(int(time.time()))

    outputdict = {}
    for i in range(times):
        list_copy = copy.deepcopy(item_list)
        random.shuffle(list_copy)

        outputdict['config_'+str(i)] = {}
        outputdict['config_'+str(i)]['bin1'] = list_copy[0:14]
        outputdict['config_'+str(i)]['bin2'] = list_copy[15:29]
    with open(output, 'w') as f:    
        json.dump(outputdict, f)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print('Useage--->number of configs     output file')
        return
    else:
        output_random_bin(int(sys.argv[1]), sys.argv[2])    