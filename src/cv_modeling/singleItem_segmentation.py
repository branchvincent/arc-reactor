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


import bpy, math, random, mathutils, os, sys, time
from mathutils import Vector

#-------------------CHANGE THE PATH---------------------------#
itemsDir = '/home/bk/Documents/code/reactor/src/cv_modeling/apc_main/object_models/tarball/'
outputPath = '/home/bk/Documents/code/reactor/src/cv_modeling/output/'
#-------------------CHANGE THE PATH---------------------------#

numImg = 600 #Number of images for a single item
Seed = 444 #Random Seed

#Import the items from .obj
def importItem(object, itemsDir):
    filepath = os.path.join(itemsDir, object) + '.obj'
    bpy.ops.import_scene.obj(filepath = filepath)
    return


def addSceneSettings(scn):
    #RenderSettings for high quality image
    rd = scn.render
    rd.resolution_percentage = 100
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
    sun_obj.location = Vector((0,0,0.25))
    sun_obj.rotation_euler = Vector((0,0,0))
    sun_obj.data.use_nodes = True
    sun_obj.data.node_tree.nodes[1].inputs[1].default_value = 4

    return

#Generate a rotation_euler in Vector
def rotGen():
    x1 = random.randrange(0,360,1)
    x2 = random.randrange(0,360,1)
    x3 = random.randrange(0,360,1)
    return Vector((x1,x2,x3))

def run():
    #Change render engine to BLENDER

    bpy.context.scene.render.engine = 'CYCLES'
    bpy.context.scene.cycles.device = 'GPU'
    bpy.data.scenes['Scene'].render.tile_x = 256
    bpy.data.scenes['Scene'].render.tile_y = 256

    #set up the material for cycles
    mat_name = 'texture_material'
    mat = bpy.data.materials.new(mat_name)
    mat.use_nodes = True
    nt = mat.node_tree
    nodes = nt.nodes
    links = nt.links
    diffuse = nodes["Diffuse BSDF"]
    texture = nodes.new("ShaderNodeTexImage")
    links.new(diffuse.inputs['Color'],   texture.outputs['Color'])
    mat.preview_render_type = 'FLAT'

    #random.seed(Seed)
    for n, obj in enumerate(objectTypes):
        importItem(obj)
        start = time.time()
        object = next(object for object in scn.objects if object.name.startswith(obj))
                
        for i in range(numImg):
            object.rotation_euler = rotGen()
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
        end = time.time()
        outstring = obj + ' is done. Took ' + str(end-start) + "\n"
        sys.stderr.write(outstring)
    return

if __name__ == '__main__':
    objectTypes = [f.replace('.obj','') for f in os.listdir(itemsDir) if ('obj' in f)&('mtl' not in f)]
    scn = bpy.data.scenes['Scene']
    addSceneSettings(scn) 
    run()
    sys.stderr.write('Rendering completed')

    
