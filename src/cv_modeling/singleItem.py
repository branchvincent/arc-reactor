#The script generate a scene with a single item with a number of
#orientation. Be sure to change the directory path of the items
#and output images before running the script.
#
#Blender version: 2.78
#Usage: run the script directly in blender (Alt + P) or type:
#
#  blender --background --python singleItem.py
#
# in terminal if you add blender to PATH.


import bpy, math, random, mathutils, os
from mathutils import Vector

#-------------------CHANGE THE PATH---------------------------#
dirPath = '/home/hugh/Downloads/apc_main/object_models/tarball'
outputPath = '/home/hugh/Pictures/arc/'
#-------------------CHANGE THE PATH---------------------------#

numImg = 5 #Number of images for a single item
Seed = 444 #Random Seed

#Import the items from .obj
def importItem(object):
    filepath = os.path.join(dirPath, object) + '.obj'
    bpy.ops.import_scene.obj(filepath = filepath)
    return


def addSceneSettings(scn):
    #RenderSettings for high quality image
    rd = scn.render
    rd.resolution_x = 256
    rd.resolution_y = 256
    rd.antialiasing_samples = '16'
    rd.use_full_sample = True
    rd.use_textures = True
    rd.use_shadows = True
    #CameraSetting
    cam = scn.objects['Camera']
    cam.location = Vector((0,0,0.5))
    cam.rotation_euler = Vector((0,0,0))
    #LightSetting
    #bpy.data.objects['Lamp'].select=True
    bpy.ops.object.delete()
    sun_data = bpy.data.lamps.new(name='sun', type='SUN')
    sun_obj = bpy.data.objects.new(name='sun', object_data=sun_data)
    scn.objects.link(sun_obj)
    sun_obj.location = Vector((0,0,0.5))
    sun_obj.rotation_euler = Vector((0,0,0))
    return

#Generate a rotation_euler in Vector
def rotGen():
    x1 = random.randrange(0,360,1)
    x2 = random.randrange(0,360,1)
    x3 = random.randrange(0,360,1)
    return Vector((x1,x2,x3))

def run():
    #Change render engine to BLENDER
    bpy.context.scene.render.engine = 'BLENDER_RENDER'
    #random.seed(Seed)
    for obj in objectTypes:
        importItem(obj)
        print(obj)
        object = next(object for object in scn.objects if object.name.startswith(obj))
                
        for i in range(numImg):
            object.rotation_euler = rotGen()
            for ob in bpy.data.objects:        
                for slot in ob.material_slots:
                    mat = slot.material
                    mat.diffuse_shader = 'LAMBERT'
                    mat.specular_shader = 'TOON'
                    mat.diffuse_color = (1,1,1)
                    mat.specular_color = (1,1,1)
                    mat.specular_intensity = 0.05
                    
            bpy.data.scenes['Scene'].render.filepath = outputPath + obj + str(i) + '.png'
            bpy.ops.render.render(write_still=True)

            for ob in bpy.data.objects:        
                for slot in ob.material_slots:
                    mat = slot.material
                    mat.diffuse_shader = 'TOON'
                    mat.diffuse_color = (0,0,0)
                    mat.specular_color = (0,0,0)
                    mat.specular_intensity = 1.00
                    mat.diffuse_toon_size = 0.00

            bpy.data.scenes['Scene'].render.filepath = outputPath + obj + '_depth_'+ str(i) + '.png'
            bpy.ops.render.render(write_still=True)
            
        object.select=True
        bpy.ops.object.delete()
        print(obj, ' is done.')
    return

if __name__ == '__main__':
    objectTypes = [f.replace('.obj','') for f in os.listdir(dirPath) if ('obj' in f)&('mtl' not in f)]
    scn = bpy.data.scenes['Scene']
    addSceneSettings(scn) 
    run()
    print('Rendering completed')
