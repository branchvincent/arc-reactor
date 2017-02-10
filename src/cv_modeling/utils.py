import bpy, math, random, mathutils
from mathutils import Vector
import os

def importItem(object, itemsDir):
    filepath = os.path.join(itemsDir, object) + '.obj'
    bpy.ops.import_scene.obj(filepath = filepath)
    return

def addSceneSettings(scn):
    #SceneData
    scn.frame_start = 10
    scn.frame_end = 10
    scn.frame_step = 1
    
    #RenderData
    rd = scn.render
    rd.fps = 24
    rd.resolution_x = 10
    rd.resolution_y = 10

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

    #Add damping to stablize the items
    obj.rigid_body.linear_damping = 0.6
    obj.rigid_body.angular_damping = 0.6
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

def locGen(dz):
    dx = random.randrange(0, 15, 1) / 100.0
    dy = random.randrange(0, 15, 1) / 100.0
    return Vector((dx,dy,dz))

def rotGen():
    x1 = random.randrange(0,360,1) / 180.0 * math.pi
    x2 = random.randrange(0,360,1) / 180.0 * math.pi
    x3 = random.randrange(0,360,1) / 180.0 * math.pi
    return Vector((x1,x2,x3))

#generate a list of vector
def vecListGen(itemsNum, vecGen, radius = 0.2, isLoc=1, dz=0):
    list = []
    for i in range(itemsNum):
        if isLoc:
            if dz==0:
                print('please input dz')
                exit()
            loc = vecGen(dz)    
            while(not isValidloc(loc, list, radius)):
                loc = vecGen(dz)
        else:
            loc = vecGen()
        list.append(loc)
    return list

