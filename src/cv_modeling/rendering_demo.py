#The script is used to automatically produce photorealistic picture with
#randomly placed items on the shelf.

import bpy, math, random, mathutils
from mathutils import Vector

Seed = 444
itemsPath = "/home/hugh/Pictures/item1.blend"
objectTypes = ["item1","item2","item3","item4","item5"]
outputPath = "/home/hugh/Pictures/blender_output/"
cameras = ["Camera"] #Add more cameras later

itemsNum = 5
dz = 0.54
radius = 0.2

def addSceneSettings(scn):
    # SceneData
    scn.frame_start = 0
    scn.frame_end = 200
    scn.frame_step = 10
    
    #RenderData
    rd = scn.render
    rd.fps = 24
    rd.resolution_x = 256
    rd.resolution_y = 256
    return

def addItemSettings(obj, loc):#, rot=Vector([0,0,0])):
    #location and rotation
    print(obj)
    bpy.data.objects[obj].select = True
    bpy.data.objects[obj].location = loc
    #bpy.data.objects[obj].location = rot
    #rigid_body properties
    bpy.ops.rigidbody.objects_add(type='ACTIVE')
    bpy.data.objects[obj].rigid_body.collision_shape="CONVEX_HULL"
    bpy.data.objects[obj].rigid_body.collision_margin = 0.0
    #Add damping to stablize the items
    bpy.data.objects[obj].rigid_body.linear_damping = 0.6
    bpy.data.objects[obj].rigid_body.angular_damping = 0.6
    return

def importItem(object):
    blendfile = itemsPath
    section = "/Object/"
    filepath = blendfile + section + object
    filename = object
    directory = blendfile + section
    bpy.ops.wm.append(filepath=filepath, filename=filename, directory=directory)
    return
    
def isValidloc(vec, list):
    if (vec[0] > 0) | (vec[1] > 0):
        # The item is not overlapped with the cube
        if vec[0]**2 + vec[1]** 2 < (radius - 0.05) **2:
            # The item is on the shelf
            for loc in list:
                dist = math.sqrt((vec[0] - loc[0])**2 +
                        (vec[1] - loc[1])**2 +
                        (vec[2] - loc[2])**2) 
                if dist < 0.05:
                    #distance btw each item is greater than 0.1
                    return False
            return True
    return False
    
def utils.locGen():
    dx = random.randrange(0, 15, 1) / 100.0
    dy = random.randrange(0, 15, 1) / 100.0
    return Vector((dx,dy,dz))

def utils.rotGen():
    x1 = random.randrange(0,360,1)
    x2 = random.randrange(0,360,1)
    x3 = random.randrange(0,360,1)
    return Vector((x1,x2,x3))

def vecListGen(itemsNum, vecGen, isLoc=1):
    list = []
    for i in range(itemsNum):
        loc = vecGen()
        print(loc)
        if isLoc:
            while(not isValidloc(loc, list)):
                loc = vecGen()
        else:
            loc = vecGen
        list.append(loc)
    return list

def run():
    #Change render engine to CYCLES
    bpy.context.scene.render.engine = 'CYCLES'

    #set the shelf active
    shelf = bpy.data.objects["ShapeIndexedFaceSet"]
    bpy.context.scene.objects.active = shelf
    shelf.select=True
    #Add the shelf to rigid_body world
    bpy.ops.rigidbody.objects_add(type='PASSIVE')
    shelf.rigid_body.collision_shape="CONVEX_HULL"
    shelf.rigid_body.collision_margin = 0.0
    shelf.rigid_body.linear_damping = 0.6
    shelf.rigid_body.angular_damping = 0.6
    
    #Placing the items
    objects = []
    phi = -90
    deg2rad = math.pi / 180
    radius = 0.1
    #random.seed(Seed)
    locs = vecListGen(itemsNum, utils.locGen, isLoc=1)
    rots = vecListGen(itemsNum, utils.rotGen, isLoc=0)
    for obj in objectTypes:
        importItem(obj)
        object = bpy.data.objects[obj]
        objects.append(object)
        object.select = False
        object.rotation_euler = utils.rotGen()
    for i,obj in enumerate(objectTypes):
        addItemSettings(obj, locs[i])#, rots[i])
    
    for cam in cameras:
        print("Rendering...")
        #bpy.ops.render.render(animation=True)

    
    for object in objects:
        object.select=True
        #bpy.ops.object.delete()
    return

if __name__ == "__main__":
    scn = bpy.data.scenes["Scene"]
    random.shuffle(objectTypes)
    addSceneSettings(scn)    
    run()
    print("Rendering completed")
