import bpy

def initShelf():
    shelf = bpy.data.objects["Shelf"]
    # add the shelf into the rigid_world if the shelf is not in it 
    if shelf.rigid_body == None:
        bpy.context.scene.objects.active = shelf
        shelf.select = True
        print("The shelf is not rigid_body, add it into rigid_world")
        bpy.ops.rigidbody.objects_add(type='PASSIVE')
        shelf.rigid_body.collision_shape = "MESH"
        shelf.rigid_body.collision_margin = 0.0
        shelf.rigid_body.linear_damping = 0.6
        shelf.rigid_body.angular_damping = 0.6
        shelf.select = False
        
    # for cube in [bpy.data.objects[str] for str in ["Cube","Cube.001", "Cube.002"]]:
    #     if cube.rigid_body == None:
    #         cube = bpy.data.objects["Cube"]
    #         bpy.context.scene.objects.active = cube
    #         cube.select = True
    #         bpy.ops.rigidbody.objects_add(type='PASSIVE')
    #         cube.rigid_body.collision_shape = "MESH"
    #         cube.rigid_body.collision_margin = 0.0
    #         cube.rigid_body.linear_damping = 0.6
    #         cube.rigid_body.angular_damping = 0.6
    #         shelf.select = False
