
// _has_include(Python.h)

#include <thread>
#include "bullet/btBulletDynamicsCommon.h"
#include "bullet/BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "bullet/BulletSoftBody/btDefaultSoftBodySolver.h"
#include "bullet/BulletSoftBody/btSoftBodyHelpers.h"
#include "bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <cassert>
#include <algorithm>
#include <tuple>

// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

// #include <GL/glut.h>

#include <windows.h> /* must include this before GL/gl.h */
#include <GL/gl.h>   /* OpenGL header file */
#include <GL/glu.h>  /* OpenGL utilities header file */
#include <stdio.h>

#include <GL/GL.h>

#pragma comment(lib, "opengl32.lib")

#ifdef EXE

#include <functional>
#include <map>
#include <vector>

#else

#define LIBRARY
#include "pybind11/pybind11.h"
#include <pybind11/stl.h>
#include <pybind11/functional.h>
namespace py = pybind11;

#endif

HWND CreateOpenGLWindow(char *title, int x, int y, int width, int height, BYTE type, DWORD flags);

void display();

class debugdrawer : public btIDebugDraw
{

public:
    int debugmode;
    bool using_gl = false;

    std::vector<std::tuple<float, float, float>> debug_vertices;

    void clear_debug_vertices()
    {
        debug_vertices.clear();
    }

    std::vector<std::tuple<float, float, float>> get_debug_vertices()
    {
        return debug_vertices;
    }

    void drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &color)
    {
        if (using_gl)
        {
            glColor3f(color.x(), color.y(), color.z());
            glBegin(GL_LINES);
            glVertex3f(from.x(), from.y(), from.z());
            glVertex3f(to.x(), to.y(), to.z());
            glEnd();
        }
        else
        {
            std::tuple<float, float, float> a(from.x(), from.y(), from.z());
            std::tuple<float, float, float> b(to.x(), to.y(), to.z());
            debug_vertices.push_back(a);
            debug_vertices.push_back(b);
        }
    }

    virtual void drawContactPoint(const btVector3 &PointOnB, const btVector3 &normalOnB, btScalar distance, int lifeTime, const btVector3 &color) {}
    virtual void reportErrorWarning(const char *warningString)
    {
        std::cout << warningString << std::endl;
    }
    virtual void draw3dText(const btVector3 &location, const char *textString){};
    virtual void setDebugMode(int debugMode)
    {
        this->debugmode = debugMode;
    }

    int getDebugMode() const override
    {
        return 1;
    }
};

enum coltype
{
    box,
    sphere,
    mesh,
    capsule
};

class rigidbody
{
public:
    rigidbody()
    {
    }

    struct frame_d
    {
        std::vector<float> matrix;
        float mass;
        btVector3 force = btVector3(0, 0, 0);
        btVector3 force_pos = btVector3(0, 0, 0);
    };

    struct rb_data
    {
        coltype type;
        std::map<int, frame_d> sim_data;
        std::vector<std::vector<float>> result_transform;
        std::vector<std::vector<float>> triangles;
        int numTris;
        float mass = 0;
        std::vector<float> matrix_transform;
        float x = 0;
        float y = 0;
        float z = 0;
        float r = 0;
        float h = 0;
    };

    // can let cpp think its out of scope and delete it
    // std::shared_ptr<rb_data> data;

    rb_data *data;

    frame_d *getormake(int frame)
    {
        if (data->sim_data.find(frame) != data->sim_data.end())
        {
            return &data->sim_data[frame];
        }
        else
        {
            frame_d f;
            data->sim_data[frame] = f;
            return &data->sim_data[frame];
        }
    }

    void settransform(std::vector<float> matrix, int frame)
    {
        frame_d *f = getormake(frame);
        f->matrix = matrix;
    }

    void setforce(float x, float y, float z, float rx, float ry, float rz, int frame)
    {
        frame_d *f = getormake(frame);
        f->force = btVector3(x, y, z);
        f->force_pos = btVector3(rx, ry, rz);
    }

    std::vector<float> gettransform(int frame)
    {
        return data->result_transform[frame];
    }

    void set_rb_rb_link()
    {
    }

    void set_config(float mass)
    {
    }

    //~rigidbody() {
    //    std::cout << "got deleted!!!" << std::endl;
    // delete data;
    //}
};

class softbody
{

public:
    struct l_data
    {
    };

    struct v_data
    {
        float l_stiff;
        float v_stiff;
        float a_stiff;
        bool has_anchor = false;
        int rb_anchor_index;
        float anchor_strength;
        bool nocoll = false;
        bool has_mat = false;
        btVector3 force = btVector3(0, 0, 0);
        bool set_pos = false;
        btVector3 pos = btVector3(0, 0, 0);
        bool has_link = false;
        int link_sb_index = 0;
        int link_node_index = 0;
        bool remove_link = false;
    };

    struct frame_d
    {
        bool has_vals = false;
        std::map<int, v_data> vertex_data;
        std::vector<float> vertex_pos;
        bool setpos = false;
        float l_stiff;
        float v_stiff;
        float a_stiff;
        float volume;
        float pressure;
        int frame;
        int pos_iter;
        float anchor_hard;
        int clusters;
        float friction;
        float damping;
        std::vector<float> matrix_transform_f;
        bool setvertextransform = false;
    };

    struct sb_data
    {
        std::vector<float> vertices;
        std::vector<std::vector<int>> faces;
        int numFaces;
        std::vector<float> matrix_transform;
        float mass = 50;
        bool constraints = true;
        std::map<int, frame_d> sim_data; // sim data per frame
        std::vector<std::vector<float>> vertex_pos_result;
    };

    // std::shared_ptr<sb_data> data;
    sb_data *data;

    softbody(std::vector<float> vertices, std::vector<std::vector<int>> faces, int numFaces, std::vector<float> matrix_transform)
    {
        data = new sb_data(); // std::shared_ptr<sb_data>(new sb_data());
        data->vertices = vertices;
        data->faces = faces;
        data->numFaces = numFaces;
        data->matrix_transform = matrix_transform;
    }

    void setvertexpos(int index, float x, float y, float z, int frame)
    {
        auto v = getormake_v(index, frame);
        v->pos = btVector3(x, y, z);
    }

    void setcontraint_rb_sb_anchor(int rb_index, int node_index, float strength, int frame, bool nocoll)
    {
        auto v = getormake_v(node_index, frame);
        v->has_anchor = true;
        v->rb_anchor_index = rb_index;
        v->anchor_strength = strength;
        v->nocoll = nocoll;
    }

    void setconstraint_sb_sb_link(int sb_index, int this_index, int other_index, int frame)
    {
        auto v = getormake_v(this_index, frame);
        v->has_link = true;
        v->link_sb_index = sb_index;
        v->link_node_index = other_index;
    }

    void removeconstraint_sb_sb_link(int index, int frame)
    {
        auto v = getormake_v(index, frame);
        v->has_link = false;
        v->remove_link = true;
    }

    void setlinkconfig()
    {
    }

    //~softbody() {
    //    std::cout << "got deleted!!!" << std::endl;
    // delete data;
    //}

    std::vector<float> getvertexposresult(int frame)
    {
        if (frame < data->vertex_pos_result.size())
        {
            return data->vertex_pos_result[frame];
        }
        else
        {
            std::cout << "no data for frame " << frame << std::endl;
            std::vector<float> v;
            return v;
        }
    }

    void setforce(float x, float y, float z, int index, int frame)
    {
        auto v = getormake_v(index, frame);
        v->force = btVector3(x, y, z);
    }

    void settransform(std::vector<float> matrix, int frame)
    {
        frame_d* f = getormake(frame);
        f->setvertextransform = true;
        f->matrix_transform_f = matrix;
    }

    frame_d *getormake(int frame)
    {
        if (data->sim_data.find(frame) != data->sim_data.end())
        {
            return &data->sim_data[frame];
        }
        else
        {
            frame_d f;
            data->sim_data[frame] = f;
            return &data->sim_data[frame];
        }
    }
    // void set_config(float l_stiff, float v_stiff, float a_stiff, float volume, float pressure, int frame)
    // data ds, float mass, float volume, float pressure, float volume_stiff, float angular_stiff, float linear_stiff, float anchorhard, int pos_iter, int clusters) {
    void set_config(float mass, float l_stiff, float v_stiff, float a_stiff, float volume, float pressure, float anchorhard, int pos_iter, int clusters, float friction, float damping, int frame)
    {
        // ONLY 5
        auto f = getormake(frame);
        f->has_vals = true;
        f->a_stiff = a_stiff;
        f->v_stiff = v_stiff;
        f->l_stiff = l_stiff;
        f->volume = volume;
        f->pressure = pressure;
        f->anchor_hard = anchorhard;
        f->clusters = clusters;
        f->pos_iter = pos_iter;
        data->mass = mass;
        f->damping = damping;
        f->friction = friction;
    }

    v_data *getormake_v(int index, int frame)
    {

        auto f = getormake(frame);
        v_data *vp;
        if (f->vertex_data.find(index) != f->vertex_data.end())
        {
            vp = &f->vertex_data[index];
        }
        else
        {
            v_data *v = new v_data;
            f->vertex_data[index] = *v;
            vp = &f->vertex_data[index];
        }
        return vp;
    }

    void setvertexconfig(int index, float l_stiff, float v_stiff, float a_stiff, int frame)
    {
        auto v = getormake_v(index, frame);
        v->l_stiff = l_stiff;
        v->v_stiff = v_stiff;
        v->a_stiff = a_stiff;
        v->has_mat = true;
    }
};

// std::unique_ptr<btSoftRigidDynamicsWorld> world;

btSoftRigidDynamicsWorld *world;
btCollisionConfiguration *collisionConfiguration;
btDispatcher *dispatcher;
btBroadphaseInterface *overlappingPairCache;
btConstraintSolver *solver;
btSoftBodySolver *softbodySolver;

bool physics_over;

btVector3 gravity;

bool draw_debug_window = false;

rigidbody mesh_rigidbody(std::vector<std::vector<float>> triangles, int numTris, float mass, std::vector<float> matrix_transform)
{
    rigidbody rb;
    rb.data = new rigidbody::rb_data(); // std::shared_ptr<rigidbody::rb_data>(new rigidbody::rb_data());
    rb.data->triangles = triangles;
    rb.data->numTris = numTris;
    rb.data->mass = mass;
    rb.data->matrix_transform = matrix_transform;
    rb.data->type = coltype::mesh;

    return rb;
}

rigidbody box_rigidbody(float x, float y, float z, float mass, std::vector<float> matrix_transform)
{
    rigidbody rb;
    rb.data = new rigidbody::rb_data(); // std::shared_ptr<rigidbody::rb_data>(new rigidbody::rb_data());
    rb.data->x = x;
    rb.data->y = y;
    rb.data->z = z;
    rb.data->mass = mass;
    rb.data->type = coltype::box;
    rb.data->matrix_transform = matrix_transform;

    return rb;
}

rigidbody sphere_rigidbody(float radius, float mass, std::vector<float> matrix_transform)
{
    rigidbody rb;
    rb.data = new rigidbody::rb_data(); // std::shared_ptr<rigidbody::rb_data>(new rigidbody::rb_data());
    rb.data->r = radius;
    rb.data->mass = mass;
    rb.data->type = coltype::sphere;
    rb.data->matrix_transform = matrix_transform;

    return rb;
}

rigidbody capsule_rigidbody(float radius, float height, float mass, std::vector<float> matrix_transform)
{
    rigidbody rb;
    rb.data = new rigidbody::rb_data(); // std::shared_ptr<rigidbody::rb_data>(new rigidbody::rb_data());
    rb.data->r = radius;
    rb.data->h = height;
    rb.data->mass = mass;
    rb.data->type = coltype::capsule;
    rb.data->matrix_transform = matrix_transform;

    return rb;
}

struct rb_destroy
{
    btCollisionShape *s;
    btMotionState *m;
    btTriangleMesh *t;
};

float phys_scale;
btTransform phys_scale_mat;
bool setscale = false;

void setworldscale(float scale) {//scale down physics data then scale the physics results up so that it doesnt affect the transform just the physics calculation
    setscale = true;
    phys_scale = scale;
    std::vector<float> vec = { scale ,0,0,0,0,scale,0,0,0,0,scale,0,0,0,0,1 };
    phys_scale_mat.setFromOpenGLMatrix(&vec[0]);
}

btRigidBody *createRididbodyFromShape(btSoftRigidDynamicsWorld *world, btCollisionShape *shape, float mass, btTransform t)
{

    btMotionState *motion = new btDefaultMotionState();

    btVector3 inertia = btVector3(0, 0, 0);
    if (mass != 0.0f)
    {
        shape->calculateLocalInertia(mass, inertia);
    }

    if (setscale) {
        shape->setLocalScaling(btVector3(phys_scale, phys_scale, phys_scale));
    }

    btRigidBody::btRigidBodyConstructionInfo info(mass, motion, shape, inertia);
    btRigidBody *rigidBody = new btRigidBody(info);

    rb_destroy *rbds = new rb_destroy();
    rbds->m = motion;
    rbds->s = shape;

    rigidBody->setUserPointer(rbds);

    world->addRigidBody(rigidBody);
    //btTransform b;
    //b.setFromOpenGLMatrix(&matrix_transform[0]);
    rigidBody->setWorldTransform(t);

    return rigidBody;
}

btRigidBody *createRigidbodyFromMesh(btSoftRigidDynamicsWorld *world, std::vector<std::vector<float>> triangles, int numTris, float mass, btTransform t)
{

    btCollisionShape *shape;

    btTriangleMesh *trimesh = new btTriangleMesh();
    for (int i = 0; i < numTris; i += 3)
    {
        btVector3 p1(triangles[i][0], triangles[i][1], triangles[i][2]);
        btVector3 p2(triangles[i + 1][0], triangles[i + 1][1], triangles[i + 1][2]);
        btVector3 p3(triangles[i + 2][0], triangles[i + 2][1], triangles[i + 2][2]);
        trimesh->addTriangle(p1, p2, p3);
    }

    if (mass == 0.0f)
    {
        shape = new btBvhTriangleMeshShape(trimesh, false);
        shape->setMargin(0.5);
    }
    else
    {
        shape = new btConvexTriangleMeshShape(trimesh, true);
    }

    // btTransform b;
    // b.setFromOpenGLMatrix(&matrix_transform[0]);
    btMotionState *motion = new btDefaultMotionState(); // new btDefaultMotionState(b);//

    btVector3 inertia = btVector3(0, 0, 0);
    if (mass != 0.0f)
    {
        shape->calculateLocalInertia(mass, inertia);
    }

    btRigidBody::btRigidBodyConstructionInfo info(mass, motion, shape, inertia);
    btRigidBody *rigidBody = new btRigidBody(info);

    rb_destroy *rbds = new rb_destroy();
    rbds->m = motion;
    rbds->s = shape;
    rbds->t = trimesh;

    rigidBody->setUserPointer(rbds);

    world->addRigidBody(rigidBody);

    //btTransform b;
    //b.setFromOpenGLMatrix(t);
    rigidBody->setWorldTransform(t);

    return rigidBody;
}

btSoftBody *createSoftbody(btSoftRigidDynamicsWorld *world, std::vector<float> vertices, std::vector<std::vector<int>> faces, int numFaces, btTransform t, float mass)
{
    // test input

    if (faces.size() == numFaces)
    {
        for (int j = 0; j < numFaces; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                if (faces[j].size() == 3)
                {
                    if (faces[j][k] > numFaces || faces[j][k] < 0)
                    {
                        std::cout << "bad indices!!!" << std::endl;

                        return nullptr;
                    }
                }
                else
                {
                    std::cout << "bad indices tupel!!!" << std::endl;

                    return nullptr;
                }
            }
        }
    }
    else
    {
        std::cout << "wrong number of faces!!!" << std::endl;

        return nullptr;
    }

    float *safe_vertices = new float[numFaces * 3];
    if (vertices.size() != numFaces * 3)
    {

        std::cout << "size inconsistent" << std::endl;
    }

    std::vector<std::tuple<float, float, float>> vert_check;

    for (int i = 0; i < vertices.size(); i++)
    {
        if (i % 3 == 0)
        {
            auto a = std::make_tuple(vertices[i], vertices[i + 1], vertices[i + 2]);
            if (std::find(vert_check.begin(), vert_check.end(), a) != vert_check.end())
            {
                std::cout << "double vertex!!!" << std::endl;

                return nullptr;
            }
            else
            {
                vert_check.push_back(a);
            }
        }
        safe_vertices[i] = vertices[i];
    }
    int **safe_faces = new int *[numFaces];
    int *safe_list_of_indicies = new int[numFaces * 3];
    for (int j = 0; j < numFaces; j++)
    {
        for (int k = 0; k < 3; k++)
        {
            safe_list_of_indicies[3 * j + k] = faces[j][k];
        }
        safe_faces[j] = &safe_list_of_indicies[3 * j];
    }

    btSoftBody *softBody = btSoftBodyHelpers::CreateFromTriMesh(world->getWorldInfo(), &safe_vertices[0], &safe_faces[0][0], numFaces);
    softBody->m_materials[0]->m_kLST = 0.45;
    softBody->m_cfg.kVC = 1000;
    softBody->setTotalMass(mass, false);
    softBody->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;
    softBody->generateClusters(17);
    softBody->setSelfCollision(true);
    softBody->m_cfg.kAHR = 1.0;
    softBody->m_cfg.piterations = 16;
    softBody->setPose(true, false);
    softBody->getWorldInfo()->m_gravity.setValue(gravity.x(), gravity.y(), gravity.z());
    world->addSoftBody(softBody);
    //btTransform b;
    //b.setFromOpenGLMatrix(&matrix_transform[0]);
    softBody->transformTo(t);

    return softBody;
}

void cleanup()
{

    delete softbodySolver;
    softbodySolver = 0;
    delete solver;
    solver = 0;
    delete overlappingPairCache;
    overlappingPairCache = 0;
    delete dispatcher;
    dispatcher = 0;
    delete collisionConfiguration;
    collisionConfiguration = 0;
}

std::mutex worldmtx;
std::condition_variable cv;
// vetted

// void runOnThread(int start, int end, float timestep, int max_substeps, std::vector<softbody> softbodies, std::vector<rigidbody> rigidbodies, std::function<void()> OnComplete, std::function<void()> OnProgress)
//{

debugdrawer *debug;

void setupphysics()
{
    // setup physics world
    // collision config
    collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

    // collision dispatcher
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    overlappingPairCache = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver;
    softbodySolver = (btSoftBodySolver *)new btDefaultSoftBodySolver();

    // btSoftRigidDynamicsWorld* world;
    world = new btSoftRigidDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration, softbodySolver); // std::unique_ptr<btSoftRigidDynamicsWorld>(new btSoftRigidDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration, softbodySolver)); // cool auto delete!!!

    debug = new debugdrawer();
    debug->using_gl = draw_debug_window;
    world->setDebugDrawer((btIDebugDraw *)debug);
    world->getDebugDrawer()->setDebugMode(1);
}

std::vector<std::tuple<float, float, float>> getdebugverts()
{
    debug->clear_debug_vertices();
    world->debugDrawWorld();
    return debug->get_debug_vertices();
}

std::vector<btSoftBody *> sbs;
std::vector<btRigidBody *> rbs;



void setdata(std::vector<softbody> softbodies, std::vector<rigidbody> rigidbodies)//warning clears data!
{
    //???
    //sbs.clear();
    //rbs.clear(); gets cleared with exit()
    world->setGravity(gravity);

    // create softbodies and rigid bodies

    // std::vector<btCollisionShape *> shapes;

    std::cout << "rbs:" << rigidbodies.size() << "sbs:" << softbodies.size() << std::endl;

    for (int i = 0; i < rigidbodies.size(); i++)
    {

        rigidbody *rb = &rigidbodies[i];

        btRigidBody *m_rigidbody;

        btTransform t;
        t.setFromOpenGLMatrix(&rb->data->matrix_transform[0]);
        if (setscale) {
            t.mult(t, phys_scale_mat);
        }

        switch (rb->data->type)
        {
        case coltype::sphere:
            m_rigidbody = createRididbodyFromShape(world, new btSphereShape(rb->data->r), rb->data->mass, t);
            break;

        case coltype::box:
            m_rigidbody = createRididbodyFromShape(world, new btBoxShape(btVector3(rb->data->x, rb->data->y, rb->data->z)), rb->data->mass, t);
            break;

        case coltype::mesh:
            m_rigidbody = createRigidbodyFromMesh(world, rb->data->triangles, rb->data->numTris, rb->data->mass, t);
            break;
        case coltype::capsule:
            m_rigidbody = createRididbodyFromShape(world, new btCapsuleShape(rb->data->r, rb->data->h), rb->data->mass, t);
            break;
        default:
            break;
        }

        rbs.push_back(m_rigidbody);

        // trigidbodies.push_back(*rb);
    }

    for (int i = 0; i < softbodies.size(); i++) // assume this goes in order..
    {

        softbody *sb = &softbodies[i];


        btTransform t;
        t.setFromOpenGLMatrix(&sb->data->matrix_transform[0]);
        if (setscale) {
            t.mult(t, phys_scale_mat);
        }

        btSoftBody *m_softbody = createSoftbody(world, sb->data->vertices, sb->data->faces, sb->data->numFaces, t, sb->data->mass);

        if (m_softbody)
        {

            // set different material
            // disabled changing materials / disabling anchors over time

            softbody::frame_d d = sb->data->sim_data[0];

            btSoftBody::Material *pm = m_softbody->appendMaterial();
            pm->m_kLST = d.l_stiff;
            pm->m_kVST = d.v_stiff;
            pm->m_kAST = d.a_stiff;

            m_softbody->m_cfg.piterations = d.pos_iter;
            m_softbody->generateClusters(d.clusters);
            m_softbody->m_cfg.kPR = d.pressure;
            m_softbody->m_cfg.kVC = d.volume;
            m_softbody->m_cfg.kAHR = d.anchor_hard;
            m_softbody->m_cfg.kDF = d.friction;
            m_softbody->m_cfg.kDP = d.damping;

            sbs.push_back(m_softbody);
        }
        // copy python objects
        // tsoftbodies = ;
    }

    // std::cout << tsoftbodies.size() << " " << world->getSoftBodyArray().size() << std::endl;

    // do the simulation while updating the stuff if something is found that frame

    // release so we dont interrupt python anymore

    // copy data from python objects
    //tsoftbodies = softbodies;
    //trigidbodies = rigidbodies;
}

// SETUP WINDOWS WINDOW MANUALLY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// HDC hDC;				/* device context */
// HGLRC hRC;				/* opengl context */
// HWND  hWnd;				/* window */
// MSG   msg;				/* message */
//
// char a[] = "my window";
//
// hWnd = CreateOpenGLWindow(a, 0, 0, 256, 256, PFD_TYPE_RGBA, 0);
// if (hWnd == NULL)
//    exit(1);
//
// hDC = GetDC(hWnd);
// hRC = wglCreateContext(hDC);
// wglMakeCurrent(hDC, hRC);
//
// setup_glut();
//
////ShowWindow(hWnd, nCmdShow);
// ShowWindow(hWnd, 3);
//
// std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//
//  END !!!!!!!!!!!!!!!!!!!!!!!

/*
std::cout << "starting Physics loop!" << std::endl;

#ifdef LIBRARY
py::gil_scoped_release release;
#endif
for (int frame = start; frame < end; frame++)
{
*/




void updatedata(std::vector<softbody> softbodies, std::vector<rigidbody> rigidbodies, int frame)//use frame 0 for realtime
{

    // update rigid bodies

    // set mass, lin. and ang. velocity, transform , forces

    for (int i = 0; i < rigidbodies.size(); i++)
    {
        rigidbody rb = rigidbodies[i];
        if (rb.data->sim_data.find(frame) != rb.data->sim_data.end())
        {
            // rb.data->sim_data[frame].matrix;

            //btTransform b;
            //b.setFromOpenGLMatrix(&rb.data->sim_data[frame].matrix[0]);

            btTransform t;
            t.setFromOpenGLMatrix(&rb.data->sim_data[frame].matrix[0]);
            if (setscale) {
                t.mult(t, phys_scale_mat);
            }

            rbs[i]->setWorldTransform(t);
            // rbs[i]->setA

            if (rb.data->sim_data[frame].force != btVector3(0, 0, 0))
            {
                rbs[i]->applyForce(rb.data->sim_data[frame].force, rb.data->sim_data[frame].force_pos);
            }
        }
    }

    // update soft bodies
    // no update vertex mat and links or yes????
    // set mass, vertex pos, pos, transform, vertex mat, settings, link settings/ mats, forces

    // std::cout << "frame:" << frame << std::endl;

    for (int i = 0; i < softbodies.size(); i++)
    {
        auto sb = softbodies[i];
        if (sb.data->sim_data.find(frame) != sb.data->sim_data.end())
        {

            if (sb.data->sim_data[frame].has_vals) {
                sbs[i]->m_cfg.kPR = sb.data->sim_data[frame].pressure;
                sbs[i]->m_cfg.kVC = sb.data->sim_data[frame].volume;
                sbs[i]->m_cfg.kDP = sb.data->sim_data[frame].damping;
                sbs[i]->m_cfg.kDF = sb.data->sim_data[frame].friction;
            }


            if (sb.data->sim_data[frame].setvertextransform) {
                btTransform t;
                t.setFromOpenGLMatrix(&sb.data->sim_data[frame].matrix_transform_f[0]);
                if (setscale) {
                   t.mult(t, phys_scale_mat);
                }
                for (int j = 0; j < sbs[i]->m_nodes.size(); j++) {
                    //sbs[i]->m_nodes[j].m_x = t * sbs[i]->m_nodes[j].m_x;
                }
                sbs[i]->transformTo(t);
                //sbs[i]->setPose(true, false);
            }


            for (auto const &[key, val] : sb.data->sim_data[frame].vertex_data)
            {
                // set anchors
                if (val.has_anchor)
                {
                    sbs[i]->appendAnchor(key, rbs[val.rb_anchor_index], val.nocoll, val.anchor_strength); // ALL IS GOOD
                }
                // set material
                if (val.has_mat)
                {
                    sbs[i]->m_nodes[key].m_material->m_kAST = val.a_stiff;
                    sbs[i]->m_nodes[key].m_material->m_kLST = val.l_stiff;
                    sbs[i]->m_nodes[key].m_material->m_kVST = val.v_stiff;
                }
                // setforces
                if (val.force)
                {
                    sbs[i]->addForce(val.force, key);
                }

                if (val.has_link)//cant remove these tho...
                {
                    sbs[i]->appendLink(&sbs[i]->m_nodes[key], &sbs[val.link_sb_index]->m_nodes[val.link_node_index]);
                }

                if (val.set_pos) {
                    sbs[i]->m_nodes[key].m_x = val.pos;
                }

                /*if (val.remove_link) {
                    btSoftBody::Link::
                    sbs[i]->m_links.findLinearSearch(btSoftBody::Link)
                }*/
            }
        }
    }
}
//{//mini scope
// std::unique_lock<std::mutex> lock(worldmtx);

// cv.wait(lock);
// releases when lock goes out of scope

void updatephysics(float timestep, int max_substeps)
{

    world->stepSimulation(timestep, max_substeps);

    //}
}
    
void updateobjects(std::vector<softbody> softbodies, std::vector<rigidbody> rigidbodies,int frame)
{
    // glutPostRedisplay();

    // std::cout << "calculated frame" << std::endl;

    // save the new data!
    for (int i = 0; i < softbodies.size(); i++)
    {
        auto btsb = sbs[i];
        std::vector<float> vertices;

        btTransform t;
        if (softbodies[i].data->sim_data[frame].matrix_transform_f.size() > 0) {
            t.setFromOpenGLMatrix(&softbodies[i].data->sim_data[frame].matrix_transform_f[0]);
            if (setscale) {
                t.mult(t, phys_scale_mat);
            }
        }
        else {
            t = sbs[i]->getWorldTransform();//????
            if (setscale) {
                t.mult(t, phys_scale_mat);
            }
        }

        for (int i = 0; i < btsb->m_nodes.size(); i++)
        {

            btVector3 v = t.inverse() * btsb->m_nodes[i].m_x;
            vertices.push_back(v.x());
            vertices.push_back(v.y());
            vertices.push_back(v.z());
        }
        softbodies[i].data->vertex_pos_result.push_back(vertices);
    }

    for (int i = 0; i < rigidbodies.size(); i++)
    {
        auto btrb = rbs[i];
        std::vector<float> matrix;
        matrix.resize(16);
        btTransform t = btrb->getWorldTransform();
        if (setscale) {
            t.mult(t, phys_scale_mat);
        }
        t.getOpenGLMatrix(&matrix[0]);

        rigidbodies[i].data->result_transform.push_back(matrix);
    }
}

// std::cout << "Physics loop Complete!" << std::endl;

// WINDOWS VIEWER CLEAN UP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// while (GetMessage(&msg, hWnd, 0, 0)) {
//    std::cout << "Getmessage" << std::endl;
//    TranslateMessage(&msg);
//    DispatchMessage(&msg);
//}
//
// wglMakeCurrent(NULL, NULL);
// ReleaseDC(hWnd, hDC);
// wglDeleteContext(hRC);
// DestroyWindow(hWnd);
//
// WINDOWS VIEWER CLEAN UP END!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// clean up

// if (false) {

void exitphysics()
{

    for (auto rb : rbs)
    {
        world->removeRigidBody(rb);
        rb_destroy *r = (rb_destroy *)rb->getUserPointer();
        delete r->m;
        delete r->s;
        delete r->t;
        delete rb->getUserPointer();
        delete rb;
    }
    rbs.clear();

    for (auto sb : sbs)
    {
        world->removeSoftBody(sb);
        delete sb;
    }
    sbs.clear();

    // dont need to delete because autopointer!!! cool
    //

    delete world;
    world = 0;

    cleanup();

    //}

    std::cout << "Clean up Complete!" << std::endl;

#ifdef LIBRARY
    py::gil_scoped_acquire aquire1;
#endif

    // softbodies = tsoftbodies;

    // rigidbodies = trigidbodies;

    // std::tuple<std::vector<std::vector<std::vector<float>>>, std::vector<std::vector<std::vector<float>>>> output;

    // for (int i = 0; i < tsoftbodies.size(); i++) {
    //     for (int j = 0; j < tsoftbodies[i].data->vertex_pos_result.size(); j++) {
    //         for (int j = 0; j < tsoftbodies[i].data->vertex_pos_result.size(); j++) {

    //        }
    //    }
    //}

    // for (int j = 0; j < rigidbodies.size(); j++) {
    //     rigidbodies[j] = trigidbodies[j];
    // }

    // so

    // std::cout << "data " << tsoftbodies[0].getvertexposresult(0).size() << std::endl;

    // softbodies[0].data->mass = 10000;

    // OnComplete();

#ifdef LIBRARY
// py::gil_scoped_release release3;
#endif

    std::cout << "Physicsdata sent!" << std::endl;

    // for (auto sb : tsoftbodies) {
    //     sb.data->vertex_pos_result
    // }
}

void runOnThread(int start, int end, float timestep, int max_substeps, std::vector<softbody> softbodies, std::vector<rigidbody> rigidbodies, std::function<void()> OnComplete, std::function<void()> OnProgress)
{
    setupphysics();
    
    #ifdef LIBRARY
    py::gil_scoped_acquire acquire;
    #endif
    std::vector tsoftobj = softbodies;
    std::vector trigidobj = rigidbodies;
    setdata(softbodies, rigidbodies);
    #ifdef LIBRARY
    py::gil_scoped_release release;
    #endif
    for (int frame = start; frame < end; frame++)
    {
        updatedata(tsoftobj, trigidobj, frame);
        updatephysics(timestep,max_substeps);
        if (draw_debug_window)
        {
        // std::cout << "drawing frame?" << std::endl;
            display();
        }
        updateobjects(softbodies,rigidbodies,frame);//not tsoftobj because it would be the same datapointer anyway
    }
    exitphysics();
}

void runSimulation(int start, int end, float timestep, int max_substeps, std::vector<softbody> softbodies, std::vector<rigidbody> rigidbodies, std::function<void()> OnComplete, std::function<void()> OnProgress)
{
    draw_debug_window = false;
    std::thread mythread(runOnThread, start, end, timestep, max_substeps, softbodies, rigidbodies, OnComplete, OnProgress);
    mythread.detach();
}

void OnComplete()
{
    std::cout << "Completed Physics" << std::endl;
    physics_over = true;
}

void OnProgress()
{
}

void setgravity(float x, float y, float z)
{
    gravity = btVector3(x, y, z);
}

void phystest()
{

    std::cout << "Starting Physics" << std::endl;

    std::vector<float> verts = {0.0, 0.0, -1.0, 0.7236073017120361, -0.5257253050804138, -0.44721952080726624, -0.276388019323349, -0.8506492376327515, -0.4472198486328125, -0.8944262266159058, 0.0, -0.44721561670303345, -0.276388019323349, 0.8506492376327515, -0.4472198486328125, 0.7236073017120361, 0.5257253050804138, -0.44721952080726624, 0.276388019323349, -0.8506492376327515, 0.4472198486328125, -0.7236073017120361, -0.5257253050804138, 0.44721952080726624, -0.7236073017120361, 0.5257253050804138, 0.44721952080726624, 0.276388019323349, 0.8506492376327515, 0.4472198486328125, 0.8944262266159058, 0.0, 0.44721561670303345, 0.0, 0.0, 1.0, -0.16245555877685547, -0.49999526143074036, -0.8506544232368469, 0.42532268166542053, -0.30901139974594116, -0.8506541848182678, 0.26286882162094116, -0.8090116381645203, -0.5257376432418823, 0.8506478667259216, 0.0, -0.5257359147071838, 0.42532268166542053, 0.30901139974594116, -0.8506541848182678, -0.525729775428772, 0.0, -0.8506516814231873, -0.6881893873214722, -0.49999693036079407, -0.5257362127304077, -0.16245555877685547, 0.49999526143074036, -0.8506544232368469, -0.6881893873214722, 0.49999693036079407, -0.5257362127304077, 0.26286882162094116, 0.8090116381645203, -0.5257376432418823, 0.9510578513145447, -0.30901262164115906, 0.0, 0.9510578513145447, 0.30901262164115906, 0.0, 0.0, -0.9999999403953552, 0.0, 0.5877856016159058, -0.8090167045593262, 0.0, -0.9510578513145447, -0.30901262164115906, 0.0, -0.5877856016159058, -0.8090167045593262, 0.0, -0.5877856016159058, 0.8090167045593262, 0.0, -0.9510578513145447, 0.30901262164115906, 0.0, 0.5877856016159058, 0.8090167045593262, 0.0, 0.0, 0.9999999403953552, 0.0, 0.6881893873214722, -0.49999693036079407, 0.5257362127304077, -0.26286882162094116, -0.8090116381645203, 0.5257376432418823, -0.8506478667259216, 0.0, 0.5257359147071838, -0.26286882162094116, 0.8090116381645203, 0.5257376432418823, 0.6881893873214722, 0.49999693036079407, 0.5257362127304077, 0.16245555877685547, -0.49999526143074036, 0.8506543636322021, 0.525729775428772, 0.0, 0.8506516814231873, -0.42532268166542053, -0.30901139974594116, 0.8506541848182678, -0.42532268166542053, 0.30901139974594116, 0.8506541848182678, 0.16245555877685547, 0.49999526143074036, 0.8506543636322021};
    std::vector<std::vector<int>> indices = {{0, 13, 12}, {1, 13, 15}, {0, 12, 17}, {0, 17, 19}, {0, 19, 16}, {1, 15, 22}, {2, 14, 24}, {3, 18, 26}, {4, 20, 28}, {5, 21, 30}, {1, 22, 25}, {2, 24, 27}, {3, 26, 29}, {4, 28, 31}, {5, 30, 23}, {6, 32, 37}, {7, 33, 39}, {8, 34, 40}, {9, 35, 41}, {10, 36, 38}, {38, 41, 11}, {38, 36, 41}, {36, 9, 41}, {41, 40, 11}, {41, 35, 40}, {35, 8, 40}, {40, 39, 11}, {40, 34, 39}, {34, 7, 39}, {39, 37, 11}, {39, 33, 37}, {33, 6, 37}, {37, 38, 11}, {37, 32, 38}, {32, 10, 38}, {23, 36, 10}, {23, 30, 36}, {30, 9, 36}, {31, 35, 9}, {31, 28, 35}, {28, 8, 35}, {29, 34, 8}, {29, 26, 34}, {26, 7, 34}, {27, 33, 7}, {27, 24, 33}, {24, 6, 33}, {25, 32, 6}, {25, 22, 32}, {22, 10, 32}, {30, 31, 9}, {30, 21, 31}, {21, 4, 31}, {28, 29, 8}, {28, 20, 29}, {20, 3, 29}, {26, 27, 7}, {26, 18, 27}, {18, 2, 27}, {24, 25, 6}, {24, 14, 25}, {14, 1, 25}, {22, 23, 10}, {22, 15, 23}, {15, 5, 23}, {16, 21, 5}, {16, 19, 21}, {19, 4, 21}, {19, 20, 4}, {19, 17, 20}, {17, 3, 20}, {17, 18, 3}, {17, 12, 18}, {12, 2, 18}, {15, 16, 5}, {15, 13, 16}, {13, 0, 16}, {12, 14, 2}, {12, 13, 14}, {13, 1, 14}};
    int numfaces = 80;
    std::vector<float> matrix = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    softbody sb(verts, indices, numfaces, matrix);

    verts = {-1.0, -1.0, -1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0};
    indices = {{0, 1, 3}, {0, 3, 2}, {2, 3, 7}, {2, 7, 6}, {6, 7, 5}, {6, 5, 4}, {4, 5, 1}, {4, 1, 0}, {2, 6, 4}, {2, 4, 0}, {7, 3, 1}, {7, 1, 5}};
    numfaces = 12;
    matrix = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 4.0, 0.0, 1.0};

    verts = {0.0, 0.0, -1.0, 0.7236073017120361, -0.5257253050804138, -0.44721952080726624, -0.276388019323349, -0.8506492376327515, -0.4472198486328125, -0.8944262266159058, 0.0, -0.44721561670303345, -0.276388019323349, 0.8506492376327515, -0.4472198486328125, 0.7236073017120361, 0.5257253050804138, -0.44721952080726624, 0.276388019323349, -0.8506492376327515, 0.4472198486328125, -0.7236073017120361, -0.5257253050804138, 0.44721952080726624, -0.7236073017120361, 0.5257253050804138, 0.44721952080726624, 0.276388019323349, 0.8506492376327515, 0.4472198486328125, 0.8944262266159058, 0.0, 0.44721561670303345, 0.0, 0.0, 1.0, -0.16245555877685547, -0.49999526143074036, -0.8506544232368469, 0.42532268166542053, -0.30901139974594116, -0.8506541848182678, 0.26286882162094116, -0.8090116381645203, -0.5257376432418823, 0.8506478667259216, 0.0, -0.5257359147071838, 0.42532268166542053, 0.30901139974594116, -0.8506541848182678, -0.525729775428772, 0.0, -0.8506516814231873, -0.6881893873214722, -0.49999693036079407, -0.5257362127304077, -0.16245555877685547, 0.49999526143074036, -0.8506544232368469, -0.6881893873214722, 0.49999693036079407, -0.5257362127304077, 0.26286882162094116, 0.8090116381645203, -0.5257376432418823, 0.9510578513145447, -0.30901262164115906, 0.0, 0.9510578513145447, 0.30901262164115906, 0.0, 0.0, -0.9999999403953552, 0.0, 0.5877856016159058, -0.8090167045593262, 0.0, -0.9510578513145447, -0.30901262164115906, 0.0, -0.5877856016159058, -0.8090167045593262, 0.0, -0.5877856016159058, 0.8090167045593262, 0.0, -0.9510578513145447, 0.30901262164115906, 0.0, 0.5877856016159058, 0.8090167045593262, 0.0, 0.0, 0.9999999403953552, 0.0, 0.6881893873214722, -0.49999693036079407, 0.5257362127304077, -0.26286882162094116, -0.8090116381645203, 0.5257376432418823, -0.8506478667259216, 0.0, 0.5257359147071838, -0.26286882162094116, 0.8090116381645203, 0.5257376432418823, 0.6881893873214722, 0.49999693036079407, 0.5257362127304077, 0.16245555877685547, -0.49999526143074036, 0.8506543636322021, 0.525729775428772, 0.0, 0.8506516814231873, -0.42532268166542053, -0.30901139974594116, 0.8506541848182678, -0.42532268166542053, 0.30901139974594116, 0.8506541848182678, 0.16245555877685547, 0.49999526143074036, 0.8506543636322021};
    indices = {{0, 13, 12}, {1, 13, 15}, {0, 12, 17}, {0, 17, 19}, {0, 19, 16}, {1, 15, 22}, {2, 14, 24}, {3, 18, 26}, {4, 20, 28}, {5, 21, 30}, {1, 22, 25}, {2, 24, 27}, {3, 26, 29}, {4, 28, 31}, {5, 30, 23}, {6, 32, 37}, {7, 33, 39}, {8, 34, 40}, {9, 35, 41}, {10, 36, 38}, {38, 41, 11}, {38, 36, 41}, {36, 9, 41}, {41, 40, 11}, {41, 35, 40}, {35, 8, 40}, {40, 39, 11}, {40, 34, 39}, {34, 7, 39}, {39, 37, 11}, {39, 33, 37}, {33, 6, 37}, {37, 38, 11}, {37, 32, 38}, {32, 10, 38}, {23, 36, 10}, {23, 30, 36}, {30, 9, 36}, {31, 35, 9}, {31, 28, 35}, {28, 8, 35}, {29, 34, 8}, {29, 26, 34}, {26, 7, 34}, {27, 33, 7}, {27, 24, 33}, {24, 6, 33}, {25, 32, 6}, {25, 22, 32}, {22, 10, 32}, {30, 31, 9}, {30, 21, 31}, {21, 4, 31}, {28, 29, 8}, {28, 20, 29}, {20, 3, 29}, {26, 27, 7}, {26, 18, 27}, {18, 2, 27}, {24, 25, 6}, {24, 14, 25}, {14, 1, 25}, {22, 23, 10}, {22, 15, 23}, {15, 5, 23}, {16, 21, 5}, {16, 19, 21}, {19, 4, 21}, {19, 20, 4}, {19, 17, 20}, {17, 3, 20}, {17, 18, 3}, {17, 12, 18}, {12, 2, 18}, {15, 16, 5}, {15, 13, 16}, {13, 0, 16}, {12, 14, 2}, {12, 13, 14}, {13, 1, 14}};
    numfaces = 80;
    matrix = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 3.0, 0.0, 1.0};

    setgravity(0, -10, 0);

    softbody sb2(verts, indices, numfaces, matrix);

    // sb2.setcontraint_rb_sb_anchor(0, 0, 1, 0,false);
    // sb2.setcontraint_rb_sb_anchor(0, 1, 1, 0,false);
    // sb2.setcontraint_rb_sb_anchor(0, 2, 1, 0,false);
    // sb2.setcontraint_rb_sb_anchor(0, 3, 1, 0,false);
    sb2.setcontraint_rb_sb_anchor(0, 4, 1, 0, false);

    // sb2.setvertexpos(10, pos);
    // sb2.data->mass = 100;
    sb2.set_config(100, 0.5, 0.5, 0.5, 100, 100, 1, 13, 13, 1, 0.5, 0);
    sb2.setforce(0, 10, 0, 1, 0);

    std::vector<std::vector<float>> triangles = {{-1.0, -1.0, -1.0}, {-0.32646918296813965, -0.32646918296813965, 1.0}, {-0.32646918296813965, 0.32646918296813965, 1.0}, {-1.0, -1.0, -1.0}, {-0.32646918296813965, 0.32646918296813965, 1.0}, {-1.0, 1.0, -1.0}, {-1.0, 1.0, -1.0}, {-0.32646918296813965, 0.32646918296813965, 1.0}, {0.32646918296813965, 0.32646918296813965, 1.0}, {-1.0, 1.0, -1.0}, {0.32646918296813965, 0.32646918296813965, 1.0}, {1.0, 1.0, -1.0}, {1.0, 1.0, -1.0}, {0.32646918296813965, 0.32646918296813965, 1.0}, {0.32646918296813965, -0.32646918296813965, 1.0}, {1.0, 1.0, -1.0}, {0.32646918296813965, -0.32646918296813965, 1.0}, {1.0, -1.0, -1.0}, {1.0, -1.0, -1.0}, {0.32646918296813965, -0.32646918296813965, 1.0}, {-0.32646918296813965, -0.32646918296813965, 1.0}, {1.0, -1.0, -1.0}, {-0.32646918296813965, -0.32646918296813965, 1.0}, {-1.0, -1.0, -1.0}, {-1.0, 1.0, -1.0}, {1.0, 1.0, -1.0}, {1.0, -1.0, -1.0}, {-1.0, 1.0, -1.0}, {1.0, -1.0, -1.0}, {-1.0, -1.0, -1.0}, {0.32646918296813965, 0.32646918296813965, 1.0}, {-0.32646918296813965, 0.32646918296813965, 1.0}, {-0.32646918296813965, -0.32646918296813965, 1.0}, {0.32646918296813965, 0.32646918296813965, 1.0}, {-0.32646918296813965, -0.32646918296813965, 1.0}, {0.32646918296813965, -0.32646918296813965, 1.0}};
    int num_triangles = 36;
    matrix = {1, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.5, 0.0, 0.0, 1.0};
    rigidbody rb1 = mesh_rigidbody(triangles, num_triangles, 0, matrix);

    matrix = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 6.0, 0.0, 1.0 };
    sb2.settransform(matrix, 10000);

    for (int i = 50; i < 10000; i++)
    {
        matrix = {1, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.5, (float)(-i * 0.0001), 0.0, 1.0};
        rb1.settransform(matrix, i);
    }

    std::vector<softbody> softbodies;
    std::vector<rigidbody> rigidbodies;

    rigidbodies.push_back(rb1);

    matrix = {1, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 1, 0.0, -1, 1.0, 0.0, 1.0};
    rigidbody rb3 = box_rigidbody(1.1, 1, 1, 1000, matrix);
    rigidbodies.push_back(rb3);

    matrix = {1, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0, 1, 0, 1.0};
    rigidbody rb6 = box_rigidbody(3, 1, 3, 0, matrix);
    rigidbodies.push_back(rb6);

    matrix = {1, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 1, 0.0, -1, 1.0, 1, 1.0};
    rigidbody rb4 = sphere_rigidbody(1, 100, matrix);
    rigidbodies.push_back(rb4);

    softbodies.push_back(sb2);

    //setworldscale(1);

    runOnThread(0, 100000, 0.0001, 16, softbodies, rigidbodies, OnComplete, OnProgress);
    // std::thread mythread(runOnThread, softbodies, rigidbodies, OnComplete, OnProgress);
    // mythread.detach();
}

void drawgrid()
{
    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_LINES);
    for (GLfloat i = -2.5; i <= 2.5; i += 0.25)
    {
        glVertex3f(i, 0, 2.5);
        glVertex3f(i, 0, -2.5);
        glVertex3f(2.5, 0, i);
        glVertex3f(-2.5, 0, i);
    }
    glEnd();
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT);
    drawgrid();

    if (world && !physics_over)
    {
        // std::unique_lock<std::mutex> lck(worldmtx);
        //
        // std::lock_guard<std::mutex> lock(worldmtx);
        // std::cout << "drew frame" << std::endl;
        world->debugDrawWorld();
        // std::cout << world->getSoftBodyArray().size() << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    else
    {
        // std::cout << "no world" << std::endl;
    }

    glFlush();

    // glMatrixMode(GL_MODELVIEW);
    // glLoadIdentity();
    // glTranslatef(0, 1, -5);
    // glRotatef(40, 1, 0, 0);
    // glRotatef(45, 0, 1, 0);

    // glutSwapBuffers();
}

void draw_update()
{
    // world->stepSimulation(0.0001, 8);
    // if (world) {
    // world->debugDrawWorld();
    //}
    // glutPostRedisplay();
}

int window = 0;

std::vector<float> view_mat = {0.6101599335670471, -0.7922782897949219, 1.345339001090906e-06, 0.0, 0.3670527935028076, 0.28268110752105713, 0.8862074613571167, 0.0, -0.7021232843399048, -0.5407277345657349, 0.46328863501548767, 0.0, -5.720523834228516, -4.565698623657227, 3.727158546447754, 1.0}; //{ 1, 0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0, 1, 0.0, -1, 1.0, 0.0, 1.0 };

bool setview = false;

void setcam(std::vector<float> matrix)
{
    setview = true;
    view_mat = matrix;
}

void setup_glut()
{
    // glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    // glutInitWindowPosition(80, 80);
    // glutInitWindowSize(800, 600);
    // window = glutCreateWindow("A Simple Tetrahedron");

    glClearColor(0.1, 0.39, 0.88, 1.0);
    glColor3f(1.0, 1.0, 1.0);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-2, 2, -1.5, 1.5, 1, 50);

    glMatrixMode(GL_MODELVIEW);

    if (setview == false)
    {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0, 1, -5);
        glRotatef(40, 1, 0, 0);
        glRotatef(45, 0, 1, 0);
    }
    else
    {

        glLoadMatrixf(&view_mat[0]);
    }

    std::cout << "Setup Glut" << std::endl;
}

void glutsetfuncs()
{
    // glutDisplayFunc(display);

#ifdef EXE
    // phystest();
#endif // EXE

    // atexit(cleanup);

    // glutIdleFunc(draw_update);//starts the loop stuff after this dont get executed!
    // literally a while true loop sadly;
    // glutMainLoop();
}

void runTestProb()
{

    std::cout << "running Test" << std::endl;
    /*

    int a = 0;
    char** b = new char* [10];

    glutInit(&a, b);
    setup_glut();


    //glutsetfuncs();

    //glutDestroyWindow(window);

    //NO GLUT
    */

    draw_debug_window = true;

    HDC hDC;   /* device context */
    HGLRC hRC; /* opengl context */
    HWND hWnd; /* window */
    // MSG   msg;				/* message */

    char a[] = "my window";

    hWnd = CreateOpenGLWindow(a, 0, 0, 460, 380, PFD_TYPE_RGBA, 0);
    if (hWnd == NULL)
        exit(1);

    hDC = GetDC(hWnd);
    hRC = wglCreateContext(hDC);
    wglMakeCurrent(hDC, hRC);

    setup_glut();

    // ShowWindow(hWnd, nCmdShow);
    ShowWindow(hWnd, 1);

    phystest();

    wglMakeCurrent(NULL, NULL);
    ReleaseDC(hWnd, hDC);
    wglDeleteContext(hRC);
    DestroyWindow(hWnd);
}

void runTestProbinThread()
{

    // need to share world object between thread and gui!
    // lock gui until physics is done
    std::cout << "running Test in thread" << std::endl;
    // draw_debug_window = true;
    // int a = 0;
    // char** b = new char* [10];

    // glutInit(&a, b);
    // setup_glut();

    std::thread mythread(runTestProb);
    mythread.detach();

    // glutsetfuncs();
}

void runTest(int start, int end, float timestep, int max_substeps, std::vector<softbody> softbodies, std::vector<rigidbody> rigidbodies, std::function<void()> OnComplete, std::function<void()> OnProgress)
{

    std::cout << "running Test" << std::endl;
    draw_debug_window = true;

    /*
    int a = 0;
    char** b = new char* [10];

    std::cout << "running Physics" << std::endl;

    glutInit(&a, b);
    */
    // setup_glut();

    // runOnThread(start, end, timestep, max_substeps, softbodies, rigidbodies, OnComplete, OnProgress);

    // glutsetfuncs();
    // runOnThread(start, end, timestep, max_substeps, softbodies, rigidbodies, OnComplete, OnProgress);

    // BETTER NO GLUT

    HDC hDC;   /* device context */
    HGLRC hRC; /* opengl context */
    HWND hWnd; /* window */
    // MSG   msg;				/* message */

    char a[] = "my window";

    hWnd = CreateOpenGLWindow(a, 0, 0, 460, 380, PFD_TYPE_RGBA, 0);
    if (hWnd == NULL)
        exit(1);

    hDC = GetDC(hWnd);
    hRC = wglCreateContext(hDC);
    wglMakeCurrent(hDC, hRC);

    setup_glut();

    // ShowWindow(hWnd, nCmdShow);
    ShowWindow(hWnd, 1);

    runOnThread(start, end, timestep, max_substeps, softbodies, rigidbodies, OnComplete, OnProgress);

    wglMakeCurrent(NULL, NULL);
    ReleaseDC(hWnd, hDC);
    wglDeleteContext(hRC);
    DestroyWindow(hWnd);

    OnComplete();
}

void runTestinThread(int start, int end, float timestep, int max_substeps, std::vector<softbody> softbodies, std::vector<rigidbody> rigidbodies, std::function<void()> OnComplete, std::function<void()> OnProgress)
{

    // need to share world object between thread and gui!

    std::cout << "running Test" << std::endl;
    // draw_debug_window = true;
    // int a = 0;
    // char** b = new char* [10];

    // std::cout << "running Physics" << std::endl;

    // glutInit(&a, b);
    // setup_glut();

    std::thread mythread(runTest, start, end, timestep, max_substeps, softbodies, rigidbodies, OnComplete, OnProgress);
    mythread.detach();

    // glutsetfuncs();
}

int main(int argc, char **argv)
{
    // correct
    // glutInit(&argc, argv);

    // for lib

    runTestProb();
}

#ifdef LIBRARY

PYBIND11_MODULE(mybinds, m)
{
    py::class_<softbody>(m, "softbody")
        .def(py::init<std::vector<float>, std::vector<std::vector<int>>, int, std::vector<float>>(), "(std::vector<float> vertices, std::vector<std::vector<int>> faces, int numFaces, std::vector<float> matrix_transform)")
        .def("set_config", &softbody::set_config, "set_config(float mass,float l_stiff, float v_stiff, float a_stiff, float volume, float pressure, float anchorhard, int pos_iter, int clusters,friction ,damping, int frame)")
        .def("setcontraint_rb_sb_anchor", &softbody::setcontraint_rb_sb_anchor, "setcontraint_rb_sb_anchor(int rb_index, int node_index, float strength)")
        .def("set_vertex_pos", &softbody::setvertexpos, "setvertexpos(int index, std::vector<float> pos)")
        .def("setvertexconfig", &softbody::setvertexconfig, "setvertexconfig(int index, float l_stiff, float v_stiff, float a_stiff)")
        .def("getvertexposresult", &softbody::getvertexposresult, "getvertexposresult(int frame)")
        .def("settransform", &softbody::settransform, "settransform(std::vector<float> matrix, int frame)");
    //.def_readwrite("name", &Pet::name)
    //.def("setName", &softbody::setName)
    //.def("getName", &softbody::getName);
    py::class_<rigidbody>(m, "rigidbody")
        //.def(py::init<std::vector<std::vector<float>>, int, float, std::vector<float>>(), "mesh_rigidbody(std::vector<std::vector<float>> triangles, int numTris, float mass, std::vector<float> matrix_transform)")
        .def("settransform", &rigidbody::settransform, "settransform(std::vector<float> matrix, int frame)")
        .def("gettransform", &rigidbody::gettransform, "gettransform(int frame)");

    m.def("mesh_rigidbody", &mesh_rigidbody, "mesh_rigidbody(std::vector<std::vector<float>> triangles, int numTris, float mass, std::vector<float> matrix_transform)");

    m.def("box_rigidbody", &box_rigidbody, "box_rigidbody(float x, float y, float z, float mass, std::vector<float> matrix_transform)");

    m.def("sphere_rigidbody", &sphere_rigidbody, "sphere_rigidbody(float radius, float mass, std::vector<float> matrix_transform)");

    m.def("capsule_rigidbody", &capsule_rigidbody, "capsule_rigidbody(float radius, float height, float mass, std::vector<float> matrix_transform)");

    m.def("setgravity", &setgravity, "setgravity(float x, float y, float z)");
    m.def("runSimulation", &runSimulation, "runSimulation(int start, int end, float timestep, int max_substeps,std::vector<softbody> softbodies, std::vector<rigidbody> rigidbodies, std::function<void()> OnComplete, std::function<void()> OnProgress)");
    m.def("runTest", &runTest, "runTest(int start, int end, float timestep, int max_substeps,std::vector<softbody> softbodies, std::vector<rigidbody> rigidbodies, std::function<void()> OnComplete, std::function<void()> OnProgress)");
    m.def("runTestProb", &runTestProb);
    m.def("runTestProbinThread", &runTestProbinThread);
    m.def("runTestinThread", &runTestinThread, "runTest(int start, int end, float timestep, int max_substeps,std::vector<softbody> softbodies, std::vector<rigidbody> rigidbodies, std::function<void()> OnComplete, std::function<void()> OnProgress)");

    m.def("setcam", &setcam, "setcam(std::vector<float> matrix)");
    m.def("setupphysics", &setupphysics);
    m.def("setdata", &setdata,"void setdata(std::vector<softbody> softbodies, std::vector<rigidbody> rigidbodies)");
    m.def("updatedata", &updatedata,"void updatedata(std::vector<softbody> softbodies, std::vector<rigidbody> rigidbodies, int frame)");
    m.def("updatephysics",&updatephysics,"updatephysics(float timestep, int max_substeps)");
    m.def("updateobjects", &updateobjects,"void updateobjects(std::vector<softbody> softbodies, std::vector<rigidbody> rigidbodies, int frame)");
    m.def("exitphysics",&exitphysics);
    m.def("getdebugverts", &getdebugverts, "std::vector<std::tuple<float, float, float>> getdebugverts()");
   
}

#endif

LONG WINAPI WindowProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    static PAINTSTRUCT ps;

    switch (uMsg)
    {
    case WM_PAINT:
        display();
        BeginPaint(hWnd, &ps);
        EndPaint(hWnd, &ps);
        return 0;

    case WM_SIZE:
        glViewport(0, 0, LOWORD(lParam), HIWORD(lParam));
        PostMessage(hWnd, WM_PAINT, 0, 0);
        return 0;

    case WM_CHAR:
        switch (wParam)
        {
        case 27: /* ESC key */
            PostQuitMessage(0);
            break;
        }
        return 0;

    case WM_CLOSE:
        PostQuitMessage(0);
        return 0;
    }

    return DefWindowProc(hWnd, uMsg, wParam, lParam);
}

HWND CreateOpenGLWindow(char *title, int x, int y, int width, int height, BYTE type, DWORD flags)
{
    int pf;
    HDC hDC;
    HWND hWnd;
    WNDCLASS wc;
    PIXELFORMATDESCRIPTOR pfd;
    static HINSTANCE hInstance = 0;

    /* only register the window class once - use hInstance as a flag. */
    if (!hInstance)
    {
        hInstance = GetModuleHandle(NULL);
        wc.style = CS_OWNDC;
        wc.lpfnWndProc = (WNDPROC)WindowProc;
        wc.cbClsExtra = 0;
        wc.cbWndExtra = 0;
        wc.hInstance = hInstance;
        wc.hIcon = LoadIcon(NULL, IDI_WINLOGO);
        wc.hCursor = LoadCursor(NULL, IDC_ARROW);
        wc.hbrBackground = NULL;
        wc.lpszMenuName = NULL;
        wc.lpszClassName = "OpenGL";

        if (!RegisterClass(&wc))
        {
            MessageBox(NULL, "RegisterClass() failed:  "
                             "Cannot register window class.",
                       "Error", MB_OK);
            return NULL;
        }
    }

    hWnd = CreateWindow("OpenGL", title, WS_OVERLAPPEDWINDOW | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
                        x, y, width, height, NULL, NULL, hInstance, NULL);

    if (hWnd == NULL)
    {
        MessageBox(NULL, "CreateWindow() failed:  Cannot create a window.",
                   "Error", MB_OK);
        return NULL;
    }

    hDC = GetDC(hWnd);

    /* there is no guarantee that the contents of the stack that become
       the pfd are zeroed, therefore _make sure_ to clear these bits. */
    memset(&pfd, 0, sizeof(pfd));
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | flags;
    pfd.iPixelType = type;
    pfd.cColorBits = 32;

    pf = ChoosePixelFormat(hDC, &pfd);
    if (pf == 0)
    {
        MessageBox(NULL, "ChoosePixelFormat() failed:  "
                         "Cannot find a suitable pixel format.",
                   "Error", MB_OK);
        return 0;
    }

    if (SetPixelFormat(hDC, pf, &pfd) == FALSE)
    {
        MessageBox(NULL, "SetPixelFormat() failed:  "
                         "Cannot set format specified.",
                   "Error", MB_OK);
        return 0;
    }

    DescribePixelFormat(hDC, pf, sizeof(PIXELFORMATDESCRIPTOR), &pfd);

    ReleaseDC(hWnd, hDC);

    return hWnd;
}

// MAKE SURE LINKER->SYSTEM->SUBSYSTEM TO WINDOWS!!!!!!!!!!!!!!!!!!!!!!!

// int APIENTRY WinMain(HINSTANCE hCurrentInst, HINSTANCE hPreviousInst, LPSTR lpszCmdLine, int nCmdShow)
//{
//     HDC hDC;				/* device context */
//     HGLRC hRC;				/* opengl context */
//     HWND  hWnd;				/* window */
//     MSG   msg;				/* message */
//
//     char a[] = "my window";
//
//     hWnd = CreateOpenGLWindow(a, 0, 0, 256, 256, PFD_TYPE_RGBA, 0);
//     if (hWnd == NULL)
//         exit(1);
//
//     hDC = GetDC(hWnd);
//     hRC = wglCreateContext(hDC);
//     wglMakeCurrent(hDC, hRC);
//
//     setup_glut();
//
//     //ShowWindow(hWnd, nCmdShow);
//     ShowWindow(hWnd, 3);
//
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//
//     //GetMessage(&msg, hWnd, 0, 0);
//
//     while (GetMessage(&msg, hWnd, 0, 0)) {
//         std::cout << "Getmessage" << std::endl;
//         TranslateMessage(&msg);
//         DispatchMessage(&msg);
//     }
//
//     wglMakeCurrent(NULL, NULL);
//     ReleaseDC(hWnd, hDC);
//     wglDeleteContext(hRC);
//     DestroyWindow(hWnd);
//
//     return msg.wParam;
// }