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
import logging

#add current working directory into $PATH
cwd = os.getcwd()

if cwd not in sys.path:
    sys.path.append(cwd)
    print(cwd + ' is added into $PATH')

import imp
import init
import utils
import intersect
imp.reload(init)
imp.reload(utils)
imp.reload(intersect)

from init import initShelf

home = '/home/hh162/'
pwd = os.path.join(home,'code/reactor/src/cv_modeling/')
itemsDir = os.path.join(home,'Documents/blender/items/apc_main/object_models/tarball')
outputDir = os.path.join(home,'Documents/blender/output/')
logPath = os.path.join(pwd, 'blender.log')
camList = ['Cam_red', 'Cam_purple', 'Cam_cyan', 'Cam_green']

imgNum = 1
itemNum = 1
dz = 0.65
radius = 0.2
    
scn = bpy.data.scenes["Scene"]
scn.frame_start = 1
scn.frame_end = 3
scn.frame_step = 1
    
#RenderData
rd = scn.render
rd.fps = 1
rd.resolution_x = 720
rd.resolution_y = 1280


def renderfrom(Cam, i, k):
    #change the active camera
    bpy.data.scenes["Scene"].camera = Cam
    bpy.data.scenes['Scene'].render.filepath = outputDir + '/run' + str(i) + '/' + 'cam' + str(k) + '/'
    bpy.ops.render.render(animation=True)

def getItemsList(itemsDir):
    return  [os.path.splitext(os.path.basename(f))[0] for f in os.listdir(itemsDir) if os.path.splitext(os.path.basename(f))[1]==".obj"]

def randomSelect(list, itemNum):
    random.shuffle(list)
    return list[:itemNum]

def run(i):
    itemsObjSel = []
    itemsSel = randomSelect(items, itemNum)  #generate a random selection of items
    #import .obj model
    
    locs = utils.vecListGen_dist(itemNum, utils.locGen, radius=radius, isLoc=1, layer=3)
    rots = utils.vecListGen_dist(itemNum, utils.rotGen, radius=radius, isLoc=0, layer=3)

    for j, item in enumerate(itemsSel):
        utils.importItem(item, itemsDir)
        obj = bpy.data.objects[item]
        itemsObjSel.append(obj)
        utils.addItemSettings(obj, locs[j], rots[j])
        utils.addMaterialSettings(item)
    logger.info("All items are configured")
    #rendering from each camera    

    for k,cam in enumerate(camList):
        renderfrom(bpy.data.objects[cam], i, k)
        logger.info("Rendering from " + cam + " is completed")

        
    manifest = [x.name for x in list(bpy.data.objects)]
    logger.info("MANIFEST: " + str(manifest))

    for obj in itemsObjSel:
        logger.info(obj.name + " to be deleted")
        if obj == None:
            return
        else:
            obj.select = True
            bpy.ops.object.delete()
            logger.info(obj.name + " deleted")
            return
        
if __name__ == '__main__':

    logger = logging.getLogger('blender')
    hdlr = logging.FileHandler(logPath)
    formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
    hdlr.setFormatter(formatter)
    logger.addHandler(hdlr)
    logger.setLevel(logging.INFO)

    bpy.context.scene.render.engine = 'BLENDER_RENDER'
    initShelf()
    items = getItemsList(itemsDir)
    for i in range(imgNum):
        run(i)
        sys.stderr.write('Rendering completed')
    
