import bpy, math, random, mathutils
from mathutils import Vector
import os

def importItem(object, itemsDir):
    filepath = os.path.join(itemsDir, object) + '.obj'
    bpy.ops.import_scene.obj(filepath = filepath)
    return

def addSceneSettings(scn):
    #SceneData
    scn.frame_start = 1
    scn.frame_end = 3
    scn.frame_step = 1
    
    #RenderData
    rd = scn.render
    rd.fps = 1
    rd.resolution_x = 720
    rd.resolution_y = 1280

def addItemSettings(obj, loc, rot=Vector([0,0,0])):

    obj.select = True
    bpy.context.scene.objects.active = obj

    #location and rotation
    obj.location = loc
    obj.rotation_euler = rot

    #rigid_body properties
    bpy.ops.rigidbody.objects_add(type='ACTIVE')
    obj.rigid_body.collision_shape="MESH"
    obj.rigid_body.collision_margin = 0.0
    obj.rigid_body.mass = 0.01
    obj.rigid_body.friction = 1
    #Add damping to stablize the items
    obj.rigid_body.linear_damping = 1
    obj.rigid_body.angular_damping = 1
    obj.select = False
    return

def addMaterialSettings(obj):
    print(obj + " material is configured")
    for slot in bpy.data.objects[obj].material_slots:
        mat = slot.material
        mat.diffuse_shader = 'LAMBERT'
        mat.specular_shader = 'COOKTORR'
        mat.diffuse_color = (1,1,1)
        mat.specular_color = (1,1,1)
        mat.specular_intensity = 0.00
        
def isValidloc(vec, list, radius):
    if vec[0]**2 + vec[1]** 2 < (radius - 0.05) **2:
            # The item is on the shelf
        if vec[0]**2 + vec[1]** 2 > 0.05:
            for loc in list:
                dist = math.sqrt((vec[0] - loc[0])**2 +
                        (vec[1] - loc[1])**2 +
                        (vec[2] - loc[2])**2) 
                if dist > 0.05:
                    #distance btw each item is greater than 0.1
                    return True
    return False

def locGen(layer):
    dx = random.randrange(0, 15, 1) / 100.0
    dy = random.randrange(0, 15, 1) / 100.0
    if layer == 3:
        dz = random.randrange(65, 100, 1) / 100.0
    elif layer == 2:
        dz = randoom.randrange(30, 40, 1) / 100.0
    elif layer == 1:
        dz = random.randrange(5, 20, 1) / 100.0
    return Vector((dx,dy,dz))

def rotGen():
    x1 = random.randrange(0,360,1) / 180.0 * math.pi
    x2 = random.randrange(0,360,1) / 180.0 * math.pi
    x3 = random.randrange(0,360,1) / 180.0 * math.pi
    return Vector((x1,x2,x3))

#generate a list of vector with uniform distance between each item
def vecListGen_dist(itemsNum, vecGen, radius = 0.2, isLoc=1, layer=3):
    list = []
    for i in range(itemsNum):
        if isLoc:
            loc = vecGen(layer)    
            while(not isValidloc(loc, list, radius)):
                loc = vecGen(layer)
        else:
            loc = vecGen()
        list.append(loc)
    return list

#So the system should detect collision btw each single object in the scene
def vecListGen_bmesh(items, shelf, radius = 0.2, isLoc=1, dz=0):
    items_existed = [shelf]
    for i, item in enumerate(items):
        obj = bpy.data.objects[item]
        obj.rotation_euler = rotGen()
        obj.location = locGen(dz)
        for item_existed in items_existed:
            while(intersect(item, item_existed)):
                obj.location = locGen()
        items_existed.append(obj)
    return

