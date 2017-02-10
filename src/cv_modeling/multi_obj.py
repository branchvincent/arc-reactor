"""
The script automatically renders a scene with the shelf and multiple items in it. It randomly select items from the itemsDir and place them on the shelf.
Please change the directory path of items and shelf before running it.

Blender version: v.2.7.8
USAGE: 
-----------
run the script in the terminal by

blender --background [SHELF_PATH] --python [SCRIPT_PATH]

UPDATE
-----------
- import other scripts as modules
"""

import bpy, math, random, mathutils, os, sys, time
from mathutils import Vector

#add current working directory into $PATH
cwd = os.getcwd()

if cwd not in sys.path:
    sys.path.append(cwd)
    print(cwd + ' is added into $PATH')

import imp
import init
import utils

imp.reload(init)
imp.reload(utils)

from init import initShelf

home = '/home/hh162/'
pwd = os.path.join(home,'code/reactor/scr/cv_modeling/')
itemsDir = os.path.join(home,'Documents/blender/items/apc_main/object_models/tarball')
outputDir = os.path.join(home,'Documents/blender/output/')

camList = ['Cam_red', 'Cam_purple', 'Cam_cyan', 'Cam_green']

imgNum = 1
itemNum = 1
dz = 0.65
radius = 0.2

def renderfrom(Cam, i, j, k):
    #change the active camera
    bpy.data.scenes["Scene"].camera = Cam
    bpy.data.scenes['Scene'].render.filepath = outputDir + str(i) + str(j) + str(k) + '.png'
    bpy.ops.render.render(animation=True)

def getItemsList(itemsDir):
    return  [os.path.splitext(os.path.basename(f))[0] for f in os.listdir(itemsDir) if os.path.splitext(os.path.basename(f))[1]==".obj"]

def randomSelect(list, itemNum):
    random.shuffle(list)
    return list[:itemNum]

def run(i):
    itemsSel = randomSelect(items, itemNum)  #generate a random selection of items
    locs = utils.vecListGen(itemNum, utils.locGen, radius, isLoc=1, dz=dz)
    rots = utils.vecListGen(itemNum, utils.rotGen, radius, isLoc=0)
    #import .obj model
    for j, item in enumerate(itemsSel):
        utils.importItem(item, itemsDir)
        print(item + " is imported")

        utils.addItemSettings(item, locs[i], rots[i])
        utils.addMaterialSettings(item)
        print("All items are configured")
    #rendering from each camera    
    for k,cam in enumerate(camList):
        renderfrom(bpy.data.objects[cam], i, j, k)
        print("Rendering from " + cam + "is completed")

    for item in itemsSel:
        bpy.data.objects[item].select =True
        #delete the items after each run.
        bpy.ops.object.delete()
        
if __name__ == '__main__':
    bpy.context.scene.render.engine = 'BLENDER_RENDER'
    scn = bpy.data.scenes["Scene"]
    utils.addSceneSettings(scn, outputDir)
    initShelf()
    items = getItemsList(itemsDir)
    for i in range(imgNum):
        run(i)
        sys.stderr.write('Rendering completed')
    
