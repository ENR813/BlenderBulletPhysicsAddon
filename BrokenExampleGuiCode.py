import bpy
import mybinds as b
from mathutils import Matrix
from multiprocessing import Process
import gpu
from gpu_extras.batch import batch_for_shader


rbs = []
rb_to_obj = {}
sbs = []
sb_to_obj = {}
world = None
ptrs = None

def make_c_rb(obj):
    matrix_flat = []
    for i in range(4):
        for j in range(4):
            matrix_flat.append(obj.matrix_world[j][i])
    rbc = b.create_box_rb(world,1,1,1,obj.mass,matrix_flat,False)
    rbs.append(rbc);
    rb_to_obj[rbc] = obj
    print("made box rb")
    return rbc

#def make_anchors(obj):

def make_rb(obj):
    mesh = obj.data
    #get triangles
    mesh.calc_loop_triangles()
    vertices = []
    num_tris = 0
    for tri in mesh.loop_triangles:
        num_tris += 3
        vertices.append([mesh.vertices[tri.vertices[0]].co[0],mesh.vertices[tri.vertices[0]].co[1],mesh.vertices[tri.vertices[0]].co[2]])
        vertices.append([mesh.vertices[tri.vertices[1]].co[0],mesh.vertices[tri.vertices[1]].co[1],mesh.vertices[tri.vertices[1]].co[2]])
        vertices.append([mesh.vertices[tri.vertices[2]].co[0],mesh.vertices[tri.vertices[2]].co[1],mesh.vertices[tri.vertices[2]].co[2]])
    #get transform
    matrix_flat = []
    for i in range(4):
        for j in range(4):
            matrix_flat.append(obj.matrix_world[j][i])
    print("before:" + str(matrix_flat))
    #make rb
    print(vertices)
    print(num_tris)
    rb = b.create_rb(world,vertices, num_tris, obj.mass, matrix_flat)
    print("after creatin:" + str(b.get_rb_transform(rb)))
    #store rb
    rbs.append(rb);
    obj_to_rb[obj] = rb
    rb_to_obj[rb] = obj
    return rb;

def make_sb(obj):
    #Not working full but good enough.... just delete double verts and remove disconnected
    mesh = obj.data
    vecs = []
    double_indexes = {}
    vertices = []
    index = 0
    for v in mesh.vertices:
        vec = [v.co[0],v.co[1],v.co[2]]
        if vecs.count(vec) == 0:
            vertices += vec
        else:
            print("hit")
            double_indexes[index] = vecs.index(vec)
        index += 1
        vecs.append(vec)
    #get indicies of same verticies and replace them all with the same index in the indices
    facedata = []
    num_faces = 0
    for tri in mesh.loop_triangles:
        num_faces += 1
        
        f = [tri.vertices[0],tri.vertices[1],tri.vertices[2]]
        for j in range(3):
            if f[j] in double_indexes:
                f[j] = double_indexes[f[j]]
                print("fixed")
        facedata.append(f)
    matrix_flat = []
    for i in range(4):
        for j in range(4):
            matrix_flat.append(obj.matrix_world[j][i])
    print(world)
    print(vertices)
    print(facedata)
    print(num_faces)
    print(matrix_flat)
    sb = b.create_sb(world, vertices, facedata, num_faces, matrix_flat)
    obj_to_sb[obj] = sb
    sb_to_obj[sb] = obj
    sbs.append(sb)
    return sb

def update_rb_props(obj):
    rb = obj_to_rb[obj]
    print("updated props")
    b.set_rb_info(rb, obj.mass, obj.contact_stiff, obj.contact_damp, obj.friction, obj.restitution, obj.hitfraction, obj.onlymass)
    b.set_rb_velocity(rb, obj.speed[0],obj.speed[1],obj.speed[2],obj.Angular_speed[0],obj.Angular_speed[1],obj.Angular_speed[2])

def update_sb_props(obj):
    sb = obj_to_sb[obj]
    b.set_sb_info(sb, obj.mass,True, obj.volume_stiff, obj.angular_stiff, obj.linear_stiff, obj.solver_iterations, obj.pressure_koeff, obj.anchorhard, obj.damping_coeff, obj.Volume_conserv_coeff, obj.friction, 16, obj.selfcol, obj.posematching, obj.contact_stiff, obj.contact_damp, obj.restitution, obj.hitfraction, obj.onlymass)

def update_rb_transforms(obj):
    rb = obj_to_rb[obj]
    a = b.get_rb_transform(rb)
    #print(a)
    M = ((a[0], a[1], a[2], a[3]),
         (a[4], a[5], a[6], a[7]),
         (a[8], a[9], a[10], a[11]),
         (a[12], a[13], a[14], a[15]))
    obj.matrix_world = M

def set_sb_transform(sb):
    obj = sb_to_obj[sb] 
    matrix_flat = []
    for i in range(4):
        for j in range(4):
            matrix_flat.append(obj.matrix_world[j][i])
    b.set_sb_Transfrom(sb, matrix_flat)
    
def set_rb_transform(obj):
    rb = obj_to_rb[obj] 
    matrix_flat = []
    for i in range(4):
        for j in range(4):
            matrix_flat.append(obj.matrix_world[j][i])
    b.set_rb_Transfrom(rb, matrix_flat)

def update_mesh(obj):
    sb = obj_to_sb[obj]
    mesh = obj.data
    mesh.calc_loop_triangles()
    verts = b.get_vertex_data(sb)  
    obj.matrix_world = Matrix.Identity(4)
    i = 0
    for v in mesh.vertices:
         mesh.vertices[i].co = (verts[i*3], verts[i*3+1], verts[i*3+2])
         i += 1

def insert_mesh_keyframe(obj):
    mesh = obj.data
    action = mesh.animation_data.action
    current_frame = bpy.context.scene.frame_current
    #add a new fcurve for every vertex and for each value of every vertex
    for v in mesh.vertices:
        fcurves = []
        for i in range(3):
            fcurves.append(action.fcurves.find("vertices[%d].co" % v.index, index = i))
        value = v.co
        #for every fcurve for the values for x y and z put in the value
        for fc, val in zip(fcurves, value):
            fc.keyframe_points.insert(current_frame, val, options={'FAST'})

def setup_mesh_keyframes(obj):
    mesh = obj.data
    action = bpy.data.actions.new("MeshAnimation")
    mesh.animation_data_create()
    mesh.animation_data.action = action
    data_path = "vertices[%d].co"
    current_frame = bpy.context.scene.frame_current
    #add a new fcurve for every vertex and for each value of every vertex
    for v in mesh.vertices:
        fcurves = []
        for i in range(3):
            fcurves.append(action.fcurves.new(data_path % v.index, index =  i))
        value = v.co
        #for every fcurve for the values for x y and z put in the value
        for fc, val in zip(fcurves, value):
            fc.keyframe_points.insert(current_frame, val, options={'FAST'})


def sim(start, end):
    for s in sbs:
        o = sb_to_obj[s]
        setup_mesh_keyframes(o)
    #clearDebug()
    b.drawdebug(world)
    for i in range(start,end):
        bpy.context.scene.frame_set(i)
        b.update(world, 0.1, 8)
        for rb in rbs:
            objr = rb_to_obj[rb]
            update_rb_transforms(objr)
            objr.keyframe_insert(data_path="location", frame=i)
            objr.keyframe_insert(data_path="rotation_euler", frame=i)
        for sb in sbs:
            objs = sb_to_obj[sb]
            update_mesh(objs)
            insert_mesh_keyframe(objs)
    bpy.context.scene.frame_set(start)
    
def sim2(start, end):
    
    global rbs, obj_to_rb, rb_to_obj, sbs ,obj_to_sb , sb_to_obj, ptrs, world
    
    #physics
    ptrs = b.setup()
    world = b.create_world(ptrs)
    
    b.setDebugDrawer(world, drawDebug)
    
    #b.set_gravity(world,0,0,-1)
    
    b.set_gravity(world, bpy.context.scene.phys_gravity[0], bpy.context.scene.phys_gravity[1],bpy.context.scene.phys_gravity[2])
    
    for obj in bpy.context.scene.objects:
            if "type" in obj:
                if obj["type"] == "rb":
                    make_rb(obj)
                if obj["type"] == "sb":
                    make_sb(obj)
                if obj["type"] == "cube_rb":
                    print("cube rb")
                    make_c_rb(obj)
    
    for s in sbs:
        o = sb_to_obj[s]
        setup_mesh_keyframes(o)
    #clearDebug()
    
    #set properties:
    
    for rb in rbs:
        objr = rb_to_obj[rb]
        #update_rb_props(objr)
    for sb in sbs:
        objs = sb_to_obj[sb]
        #update_sb_props(objs)
    
    
    b.drawdebug(world)
    for i in range(start,end):
        bpy.context.scene.frame_set(i)
        #b.set_gravity(world, bpy.context.scene.phys_gravity[0], bpy.context.scene.phys_gravity[1],bpy.context.scene.phys_gravity[2])
        b.update(world, bpy.context.scene.phys_time_step, bpy.context.scene.phys_max_sub_steps)
        for rb in rbs:
            objr = rb_to_obj[rb]
            if objr.animated == True:
                set_rb_transform(objr)
                #update_rb_props(objr)
            update_rb_transforms(objr)
            objr.keyframe_insert(data_path="location", frame=i)
            objr.keyframe_insert(data_path="rotation_euler", frame=i)
        for sb in sbs:
            objs = sb_to_obj[sb]
            if objs.animated == True:
                update_sb_props(objs)
            update_mesh(objs)
            insert_mesh_keyframe(objs)
    bpy.context.scene.frame_set(start)
    
    #clean up
    for rb in rbs:
        objr = rb_to_obj[rb]
        delete_rb(objr)
    for sb in sbs:
        objs = sb_to_obj[sb]
        delete_sb(objs)
    
    b.delete_world(world)
    b.exit(ptrs)
    
    rbs = []
    obj_to_rb = {}
    rb_to_obj = {}
    sbs = []
    obj_to_sb = {}
    sb_to_obj = {}
    


def exists(obj):
    for o in bpy.context.scene.objects:
        if o == obj:
            return True;
    return False

class bake(bpy.types.Operator):
    bl_idname = "phy.bake"
    bl_label = "Bake Physics"
    def execute(self, context):
        #clean up
        
        #print(rbs)
        #print(sbs)
        
        #for rb in rbs:
        #    objr = rb_to_obj[rb]
        #    if not exists(objr):
        #        delete_rb(objr)
        #for sb in sbs:
        #    objs = sb_to_obj[sb]
        #    if not exists(objs):
        #        delete_sb(objs)
        
        #for rb in rbs:
        #    objr = rb_to_obj[rb]
        #    update_rb_props(objr)
        
        start = bpy.context.scene.phys_start
        
        end = bpy.context.scene.phys_end
        
        sim2(start, end)
        
        #t = Process(target=sim2, args=[start, end])
        #t.start()
        return {'FINISHED'}

class makesb(bpy.types.Operator):
    bl_idname = "phy.makesb"
    bl_label = "Make Softbody"

    def execute(self, context):
        obj = bpy.context.active_object
        #if "type" in obj:
        #    if obj["type"] == "rb":
        #        delete_rb(obj)
        obj["type"] = "sb";
        #make_sb(obj)
        return {'FINISHED'}
    
class remove(bpy.types.Operator):
    bl_idname = "phy.remove"
    bl_label = "Remove Physics"

    def execute(self, context):
        obj = bpy.context.active_object
        #if "type" in obj:
        #    if obj["type"] == "rb":
        #        delete_rb(obj)
        #    if obj["type"] == "sb":
        #        delete_sb(obj)
        obj["type"] = None;
        return {'FINISHED'}



class make_cube_rb(bpy.types.Operator):
    bl_idname = "phy.make_cube_rb"
    bl_label = "Make Box RigidBody"
    def execute(self, context):
        bpy.ops.mesh.primitive_cube_add()
        obj = bpy.context.active_object
        obj["type"] = "cube_rb";
        return {'FINISHED'}

class makerb(bpy.types.Operator):
    bl_idname = "phy.makerb"
    bl_label = "Make RigidBody"

    def execute(self, context):
        obj = bpy.context.active_object
        #make_rb(obj)
        #if "type" in obj:
        #    if obj["type"] == "sb":
        #        delete_sb(obj)
        obj["type"] = "rb";
        return {'FINISHED'}


class objdatapanel (bpy.types.Panel):
    
    #bl_idname = "obj_phys_settings"
    #bl_label = 'Phyiscs Panel'
    #bl_space_type = 'VIEW_3D'
    #bl_region_type = 'UI'
    
    #bl_context = "OBJECT"
    
    bl_label = "Physics"
    bl_idname = "phy_data"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Bullet Physics"
    
    #bl_region_type = 'WINDOW'
    
    #bl_options = {'DEFAULT_CLOSED'}
    
    #@classmethod
    #def poll(cls, context):
    #    return (context.selected_objects) == 1
    
    my_float: bpy.props.FloatProperty(name="Float")
    my_bool: bpy.props.BoolProperty(name="Toggle Option")
    my_string: bpy.props.StringProperty(name="String Value")
    
    def draw(self, context):
        layout = self.layout
        obj = context.active_object
        if len(context.selected_objects) >= 1:
            if "type" in obj:
                if obj["type"] == "sb":
                    layout.label(text="Softbody")
                    layout.operator("phy.makerb")
                    for p in props3:
                        row = layout.row()
                        if p[0] == "posematching":
                            layout.label(text="Warning No Idea What these really do leave onylmass on to keep default")
                        if p[0] == "onlymass":
                            layout.label(text="Set Velocity:")
                        row.prop(obj, p[0])
                    layout.prop_search(obj,"anchors",obj,"vertex_groups")
                    layout.prop(obj,"anchor_obj")
                    
                if obj["type"] == "rb" or obj["type"] == "cube_rb":
                    layout.label(text="Rigidbody")
                    layout.operator("phy.makesb")
                    for p in props_rb:
                        row = layout.row()
                        if props3[p][0] == "friction":
                            layout.label(text="Warning No Idea What these really do leave onylmass on to keep default")
                        if props3[p][0] == "onlymass":
                            layout.label(text="Set Velocity:")
                        row.prop(context.active_object, props3[p][0])
                if obj["type"] == None:
                    layout.operator("phy.makesb")
                    layout.operator("phy.makerb")
                else:
                    layout.operator("phy.remove")
            else:
                layout.operator("phy.makesb")
                layout.operator("phy.makerb")
        else:
            layout.operator("phy.make_cube_rb")
        layout.label(text="Bake Physics")
        layout.prop(context.scene,"phys_start",text="start")
        layout.prop(context.scene,"phys_end",text="end")
        layout.prop(context.scene,"phys_gravity")
        layout.prop(context.scene,"phys_max_sub_steps")
        layout.prop(context.scene,"phys_time_step")
        layout.operator("phy.bake")


props3 = [
('mass', 0, -1, 'Float', 200.0),
('volume_stiff', 0, 1, 'Float', 0.9),
('angular_stiff', 0, 1, 'Float', 0.1),
('linear_stiff', 0, 1, 'Float', 0.1),
('solver_iterations', 0, -1, 'Int', 8), 
('pressure_koeff', 0, -1, 'Float', 20.0),
('anchorhard', 0, 1, 'Float', 0.5),
('damping_coeff', 0, -1, 'Float', 0.5),
('Volume_conserv_coeff', 0, -1, 'Float', 0.8),
('friction', 0, -1, 'Float', 0.1), 
('selfcol', 0, -1, 'Bool', True),
('posematching', 0, 1, 'Float', 0.1),
('contact_stiff', 0, -1, 'Float', 0.0), 
('contact_damp', 0, -1, 'Float', 0.0),
('restitution', 0, -1, 'Float', 0.1),
('hitfraction', 0, -1, 'Float', 0.1),
('onlymass', 0, -1, 'Bool', True),
('speed', 0, -1, 'FloatVector', (0, 0, 0)),
('Angular_speed', 0, -1, 'FloatVector', (0, 0, 0)),
('animated', 0, -1, 'Bool', False)
]

props_rb = [0,9,12,13,14,15,16,17,18,19]

#props_min = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-10000,-10000,0,0,0,0,0,0,0]

#props_max = [10000000,1,1,1,1,1,1,1,1,1,1,1,1,1,1,10000,10000,1,1,1,1,1,1,1]

classes = [make_cube_rb,makerb,makesb, objdatapanel, remove, bake]

def update_props(self, context):
    if self["type"] == "rb":
        print(self.mass)
        #update_rb_props(self)
    return None

def search(self, context, edit_text):
    list = []
    for i in range(len(context.vertex_groups)):
        list.appeng(context.vertex_groups[i].name)
    return list
    
def register():
    
    test = []
    
    for (p,min,max,type,df) in props3:
        test.append((p,min,max,type,df))
        if type == "int" or type == "Float":
            if max == -1:
                exec(f"bpy.types.Object.{p} = bpy.props.{type}Property(min={min},default={df}, update=update_props)")
            else:
                exec(f"bpy.types.Object.{p} = bpy.props.{type}Property(min={min},max={max},default={df}, update=update_props)")
        else:
            exec(f"bpy.types.Object.{p} = bpy.props.{type}Property(update=update_props,default={df})")
    
    print(test)
    
    bpy.types.Object.anchors = bpy.props.StringProperty(name="anchor_group")
    bpy.types.Object.anchor_obj = bpy.props.PointerProperty(type=bpy.types.Object)
    
    bpy.types.Scene.phys_start = bpy.props.IntProperty(min=0, default=0);
    bpy.types.Scene.phys_end = bpy.props.IntProperty(min=0, default=100);
    
    bpy.types.Scene.phys_max_sub_steps = bpy.props.IntProperty(min=0, default=8);
    bpy.types.Scene.phys_time_step = bpy.props.FloatProperty(min=0, default = 0.1);
    
    bpy.types.Scene.phys_gravity = bpy.props.FloatVectorProperty();
    
    for c in classes:
        bpy.utils.register_class(c)
    
    
def unregister():
    for p in props:
        exec(f"del bpy.types.Scene.{p}")
    bpy.utils.unregister_class(objdatapanel)
    #physics

if __name__ == "__main__":
    #unregister()
    register()