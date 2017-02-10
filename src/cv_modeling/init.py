import bpy

def initShelf():
    shelf = bpy.data.objects["Shelf"]
    bpy.context.scene.objects.active = shelf
    shelf.select = True
    bpy.ops.rigidbody.objects_add(type='PASSIVE')
    shelf.rigid_body.collision_shape = "CONVEX_HULL"
    shelf.rigid_body.collision_margin = 0.0
    shelf.rigid_body.linear_damping = 0.6
    shelf.rigid_body.angular_damping = 0.6
    for cube in ["Cube","Cube.001", "Cube.002"]:
        cube = bpy.data.objects["Cube"]
        bpy.context.scene.objects.active = cube
        cube.select = True
        bpy.ops.rigidbody.objects_add(type='PASSIVE')
        cube.rigid_body.collision_shape = "CONVEX_HULL"
        cube.rigid_body.collision_margin = 0.0
        cube.rigid_body.linear_damping = 0.6
        cube.rigid_body.angular_damping = 0.6
    
