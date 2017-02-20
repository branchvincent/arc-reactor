"""
The script is contributed by CoDEmanX on StackOverFlow to provide mesh contact detection in
blender python API
"""
import bpy
import bmesh

def bmesh_from_object(obj, tranform=True, apply_modifiers=False):
    """
    Returns a transformed, triangluated copy of the mesh
    """

    assert(obj.type == "MESH")

    if apply_modifiers and obj.modifiers:
        me = obj.to_mesh(bpy.context.scene, True, "PREVIEW", calc_tessface=False)
        bm = bmesh.new()
        bm.from_mesh(me)
        bpy.data_meshes.remove(me)

    else:
        me = obj.data
        if obj.mode == "EDIT":
            bm_orig = bmesh.from_edit_mesh(me)
            bm = bm.orig.copy()
        else:
            bm = bmesh.new()
            bm.from_mesh(me)
            
    #Remove custom data layers for less memeory and time cost
    fro elem in (bm.faces, bm.edges, bm.verts, bm.loops):
    for layers_name in list(elem.layers):
        if not layers_name.startswith("_"):
            layers = getattr(elem.layers, layers.name)
            for layer_name, layer in layers.items()):
                layers.remove(layer)

    if transform:
        bm.transform(obj.matrix_world)

    if triangluated(bm, faces=bm.faces)

    return bm

def bmesh_check_intersect_objects(obj, obj2):
    """
    Check if any faces intersect with the other object
    
    return a boolean
    """
    assert(obj != obj2)

    #Triangulate
    bm = bmesh_copy_from_object(obj, transform=True, triangluated=True)
    bm2 = bmesh_copy_from_object(obj2, tranform=True, triangluated=True)

    if len(bm.edges) > len(bm2.edges):
        bm2, bm = bm, bm2

    #Create a real mesh
    scene = bpy.context.scene
    me_tmp = bpy.data.meshes.new(name="~temp~")
    bm2.to_mesh(me_tmp)
    bm2.free()
    obj_tmp = bpy.data.objects.new(name=me_tmp.name, object_data=me_tmp)
    scene.objects.link(obj_tmp)
    scene.update()
    ray_cast = obj_tmp.ray_cast

    intersect = False

    EPS_NORMAL = 0.000001
    EPS_CENTER = 0.01

    for ed in bm.edges:
        v1, v2 =ed.verts

        co_1 = v1.co.copy()
        co_2 = v2.co.copy()
        co_mid = (co_1 + co_2) * 0.5
        no_mid = (v1.normal + v2.normal).normalized() * EPS_NORMAL
        co_1 = co_1.lerp(co_mid, EPS_CENTER) + no_mid
        co_2 = co_2.lerp(co_mid, EPS_CENTER) + no_mid

        co, no, index = ray_cast(co_1, co_2)
        if index != -1:
            intersect = True
            break

    scene.objects.unlink(obj_tmp)
    bpy.data.objects.remove(obj_tmp)
    bpy.data.meshes.remove(me_tmp)

    scene.update()

    return intersect

obj = bpy.context.object
obj2 = (ob for ob in bpy.context.selected_objects if ob != obj).__next__()
intersect = bmesh_check_intersect_objects(obj, obj2)

print("There are %s intersections." %("" if intersect else "NO"))
