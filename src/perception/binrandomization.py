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
        outputdict['config_'+str(i)]['bin1'] = list_copy[0:10]
        outputdict['config_'+str(i)]['bin2'] = list_copy[10:22]
        outputdict['config_'+str(i)]['bin3'] = list_copy[22:32]
    with open(output, 'w') as f:    
        json.dump(outputdict, f)


def showbins(file):
    with open(file, 'r') as f:
        x = json.load(f)

    for i in range(200):
        bin1 = x["config_" + str(i)]['bin1']
        bin2 = x["config_" + str(i)]['bin2']
        bin3 = x["config_" + str(i)]['bin3']
        spacing = " "*30
        print("Bin1" + spacing + "Bin2" + spacing + "Bin3")
        print("-"*68)
        for n in range(len(bin2)):
            #always start at 35
            if n < 10:
                spacing = " "*(34-len(bin1[n]))
                spacing2 = " "*(68-len(bin1[n])-len(bin2[n])-len(spacing))  
                print(bin1[n] + spacing + bin2[n] + spacing2+ bin3[n])
            else:
                print(35*" " + bin2[n] )
            
        
        print("")
        print("")
        input("press key for next order")
        print("")

if __name__ == "__main__":
    # if len(sys.argv) < 3:
    #     print('Useage--->number of configs     output file')
    # else:
    # output_random_bin(int(sys.argv[1]), sys.argv[2])    
    showbins(sys.argv[1])