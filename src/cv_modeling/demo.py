#The script is used to automatically produce photorealistic picture with
#randomly placed items on the shelf.
#The file path shall be changed before running it.
#
import bpy, math, random, mathutils
from mathutils import Vector

Seed = 444
itemsPath = "/home/hugh/Pictures/item1.blend"
objectTypes = ("item1","item2","item3","item4") 
outputPath = "/home/hugh/Pictures/blender_output/"
cameras = ["Camera"] #Add more cameras later

def addSceneSettings(scn):
    # SceneData
    scn.frame_start = 0
    scn.frame_end = 50
    scn.frame_step = 10
    
    #RenderData
    rd = scn.render
    rd.fps = 24
    rd.resolution_x = 256
    rd.resolution_y = 256
    return

def addItemSettings(obj, pos):
    #location
    bpy.data.objects[obj].location = pos
    #rigid_body prop
    bpy.ops.rigidbody.objects_add(type='ACTIVE')
    bpy.data.objects[obj].rigid_body.collision_shape="MESH"
    bpy.data.objects[obj].rigid_body.collision_margin = 0.0
    return

def importItem(object):
    blendfile = itemsPath
    section = "/Object/"
    filepath = blendfile + section + object
    filename = object
    directory = blendfile + section
    bpy.ops.wm.append(filepath=filepath, filename=filename, directory=directory)
    return
    
def run():
    #Change render engine to CYCLES
    bpy.context.scene.render.engine = 'CYCLES'

    #set the shelf active
    shelf = bpy.data.objects["ShapeIndexedFaceSet"]
    bpy.context.scene.objects.active = shelf
    shelf.select=True
    #Add the shelf to rigid_body world
    bpy.ops.rigidbody.objects_add(type='PASSIVE')
    shelf.rigid_body.collision_shape="MESH"
    shelf.rigid_body.collision_margin = 0.0

    #Placing the items
    objects = []
    phi = -90
    deg2rad = math.pi / 180
    radius = 0.1
    random.seed(Seed)
    for obj in objectTypes:
        importItem(obj)
        object = bpy.data.objects[obj]
        objects.append(object)
        phi += 40  #The items are uniformly placed on the shelf
        dx = math.cos(phi * deg2rad) * radius
        dy = math.sin(phi * deg2rad) * radius
        dz = 0.57 # The items are slightly above the shelf
        pos = (dx,dy,dz)
        addItemSettings(obj,pos)    
    
    for cam in cameras:
        bpy.ops.render.render(animation=True)
    
    for object in objects:
        object.select=True
        bpy.ops.object.delete()
    return

if __name__ == "__main__":
    scn = bpy.data.scenes["Scene"]
    addSceneSettings(scn)    
    run()
    print("Rendering completed")