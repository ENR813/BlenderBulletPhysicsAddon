import bpy
import gpu
from gpu_extras.batch import batch_for_shader
from time import sleep
from threading import Thread
import mybinds as b
import time
from mathutils import Matrix
#https://docs.blender.org/api/current/gpu.html



rbs = []
rb_to_obj = {}

sbs = []
sb_to_obj = {}

def create_sb(obj, sbs, sb_to_obj):
    mesh = obj.data
    mesh.calc_loop_triangles()
    
    #sb data
    vecs = []
    vertices = []
    index = 0
    double_indexes = []
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
            
    #matrix = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 4.0, 1.0];
    sb = b.softbody(vertices, facedata, num_faces, matrix_flat)
    
    print(vertices)
    print(facedata)
    print(num_faces)
    print(matrix_flat)
    
    sbs.append(sb)
    sb_to_obj[sb] = obj
    return sb

def create_rb(obj):
        
    mesh = obj.data
    mesh.calc_loop_triangles()
    
    
    #rb data
    vertices = []
    num_tris = 0
    for tri in mesh.loop_triangles:
        num_tris += 3
        vertices.append([mesh.vertices[tri.vertices[0]].co[0],mesh.vertices[tri.vertices[0]].co[1],mesh.vertices[tri.vertices[0]].co[2]])
        vertices.append([mesh.vertices[tri.vertices[1]].co[0],mesh.vertices[tri.vertices[1]].co[1],mesh.vertices[tri.vertices[1]].co[2]])
        vertices.append([mesh.vertices[tri.vertices[2]].co[0],mesh.vertices[tri.vertices[2]].co[1],mesh.vertices[tri.vertices[2]].co[2]])


    matrix_flat = []
    for i in range(4):
        for j in range(4):
            matrix_flat.append(obj.matrix_world[j][i])
            
    print(vertices)
    print(num_tris)
    
    rb = b.mesh_rigidbody(vertices, num_tris, 0, matrix_flat)
    rbs.append(rb)
    rb_to_obj[rb] = obj
    return rb



obj = bpy.context.selected_objects[0] #out here so its thread safe


def setup(sbs,rbs,sb_to_obj,rb_to_obj):
    b.setgravity(0,0,-10)
    sb = create_sb(obj,sbs, sb_to_obj)
    sb.set_config(800, 0.0, 0.0, 100.0, 2000, 1000, 1, 13, 13, 0.4,0.01,0);
    rbs.append(b.box_rigidbody(3,3,1,0,[1, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0, 0, -1, 1.0]))
    b.setupphysics()
    b.setdata(sbs,rbs)

setup(sbs,rbs,sb_to_obj,rb_to_obj)

shader = gpu.shader.from_builtin('3D_UNIFORM_COLOR')
coords = b.getdebugverts()
batch = batch_for_shader(shader, 'LINES', {"pos": coords})

def draw():

    b.updatedata(sbs,rbs,0)
    b.updatephysics(0.1,16)
    coords = b.getdebugverts()
    
    batch = batch_for_shader(shader, 'LINES', {"pos": coords})
    
    
    shader.uniform_float("color", (1, 1, 0, 1))
    batch.program_set(shader)
    batch.draw(shader)

screen = bpy.context.screen #out here so its thread safe

def wait():#dont handel stuff in threads it sucks
    while True:
        while not screen.is_animation_playing:
            sleep(0)
        handler = bpy.types.SpaceView3D.draw_handler_add(draw, (), 'WINDOW', 'POST_VIEW')
        while screen.is_animation_playing:
            sleep(0)
        bpy.types.SpaceView3D.draw_handler_remove(handler,'WINDOW' )
        b.exitphysics()
        rbs = []
        sbs = []
        sb_to_obj = {}
        rb_to_obj = {}
        setup(sbs,rbs,sb_to_obj,rb_to_obj)

t = Thread(target=wait,args = ())# threading does work just not when its computationally expensive i guess
t.start()
