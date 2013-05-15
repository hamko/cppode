#include <vector>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <drawstuff/texturepath.h>

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

class PartsFactory {
public:
    PartsFactory(void){};
    virtual Sphere* createSphere(World* world, double m, double r) = 0;
    virtual Capsule* createCapsule(World* world, double m, double l, double r) = 0;
    virtual WorldBuilder* createWorldBuilder(void) = 0;
    virtual World* createWorld(int id) = 0;
};
class StandardPartsFactory : PartsFactory{
public:
    StandardPartsFactory(void){};
    virtual Sphere* createSphere(World* world, double m, double r){return new Sphere(world, m, l, r);}
    virtual Capsule* createCapsule(World* world, double m, double l, double r){return new Capsule(world, m, l, r);}
    virtual WorldBuilder* createWorldBuilder(void){return new WorldBuilder();};
    virtual World* createWorld(int id){return new World(id);};
};
PartsFactory* g_factory; // singleton

class Object {
};

class Joint : public Object {
};

class GeomBody : public Object{
private:
    gGeomID m_geom;
    gBodyID m_body;
    dMass m_mass;
public:
    setBodyPosition(double x, double y, double z) {dBodySetPosition(m_body, x, y, z);}
};

class Sphere : public GeomBody {
private:
    double m_m;
    double m_r;
public:
    Sphere(World* world, double m, double r) : m_m(m), m_r(r) {
        m_body = dBodyCreate(world->getWorld());
        dMassSetSphereTotal (&m_mass, m, r);
        dBodySetMass (m_body, &m_mass);
        m_geom = dCreateSphere(world->getSpace(), r);
        dGeomSetBody (m_geom, m_body);
    }
    double getR(void){return m_r}
    double getM(void){return m_m}
    void setR(double r){m_r = r}
    void setM(double m){m_m = m}
};

class Capsule : public GeomBody {
private:
    double m_m;
    double m_r;
    double m_l;
public:
    Capsule(World* world, double m, double r, double l) : m_m(m), m_r(r), m_l(l) {
        m_body = dBodyCreate(world->getWorld());
        dMassSetCapsuleTotal (&m_mass, m, 3, r, l);
        dBodySetMass (m_body, &m_mass);
        m_geom = dCreateCapsule(world->getSpace(), r, l);
        dGeomSetBody (m_geom, m_body);
    }
    double getR(void){return m_r}
    double getL(void){return m_l}
    double getM(void){return m_m}
    void setR(double r){m_r = r}
    void setL(double l){m_l = l}
    void setM(double m){m_m = m}
};

class World {
private:
    dWorldID m_world;
    dSpaceID m_space;
    dJointGroupID m_contactgroup;
    dGeomID m_ground;
    vector<Object*> m_objs;
public:
    World(int id) {
        dInitODE2(id);
        m_world = dWorldCreate();
        m_space = dHashSpaceCreate(id);
        m_contactgroup = dJointGroupCreate(id);
        m_dWorldSetGravity(m_world,0,0,-9.8);
        m_ground = dCreatePlane(m_space,0,0,1,0);
    };
    void addObject(Object* obj) {
        m_objs.push_back(obj);
    }
    dWorldID getWorld(void){return m_world}
    dSpaceID getSpace(void){return m_space}
    dGeomID getGround(void){return m_ground}
};

class WorldBuilder {
    World* m_world;
    int m_id;
public:
    virtual void buildWorld(void);
    virtual void buildCapsule(void);
    WorldBuilder(int id) m_id(id){};
    World* getWorld(void){return m_world};
    void buildWorld(void) {m_world = g_factory->createWorld(m_id);}
    void buildCapsule(double m, double l, double r, double x, double y, double z){
        Capsule* p = g_factory->createCapsule(m_world->getWorld(), m, l, r);
        p->setBodyPosition(x, y, z);
        m_world->addObject((Object*)p);
    }
    void buildSphere(double m, double r, double x, double y, double z){
        Capsule* p = g_factory->createSphere(m_world->getWorld(), m, r);
        p->setBodyPosition(x, y, z);
        m_world->addObject((Object*)p);
    }
};

class ODESimlator {
    WorldBuilder* m_world_builder;
    dsFunctions m_fn;
    int m_argc;
    char** m_argv;
    void setFn(void (*start)(void), void (*simLoop)(int), void (*command)(int)); // setup pointers to drawstuff callback functions
public:
    ODESimlator(int argc, char** argv) : m_argc(argc), m_argv(argv) {m_world_builder = g_factory->createWorldBuilder()}
    void createWorld(void);
    void startSimulation(void); // run simulation
};
World* ODESimlator::createWorld(void)
{
    m_world_builder->buildWorld();

    m_world_builder->buildCapsule(1.0, 0.1);
    m_world_builder->buildSphere(1.0, 1.0, 0.25);

    return m_world_builder->getWorld();
}
void ODESimlator::setFn(void (*start)(void), void (*simLoop)(int), void (*command)(int)) {
    m_fn.version = DS_VERSION;
    m_fn.start = &start;
    m_fn.step = &simLoop;
    m_fn.command = &command;
    m_fn.stop = 0;
    m_fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
};
void ODESimlator::startSimulation(void)
{
    dsSimulationLoop(argc, argv, 640, 480, &fn);
}

/*
static dJointID joint[10];						// •¨‘Ì‚Ìw‘¶Ýx
*/

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
    int i,n;

    const int N = 10;
    dContact contact[N];
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    if (n > 0) {
        for (i=0; i<n; i++) {
            contact[i].surface.mode = dContactBounce;
            contact[i].surface.mu = dInfinity;
            contact[i].surface.bounce = 0.1;
            contact[i].surface.bounce_vel = 0.1;
            contact[i].surface.slip1 = 0.1;
            contact[i].surface.slip2 = 0.1;
            contact[i].surface.soft_erp = 0.5;
            contact[i].surface.soft_cfm = 0.3;
            dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
            dJointAttach (c,
                    dGeomGetBody(contact[i].geom.g1),
                    dGeomGetBody(contact[i].geom.g2));
        }
    }
}


// start simulation - set viewpoint
static void start()
{
    static float xyz[3] = {0.8317f,-0.9817f,0.8000f};
    static float hpr[3] = {101.0000f,-27.5000f,0.0000f};
    dsSetViewpoint (xyz,hpr);
}


// called when a key pressed
static void command (int cmd)
{
    switch (cmd) {
        case 'a': case 'A':
            printf("test\n");
            break;
    }
}

// simulation loop
static void simLoop (int pause)
{
    if (!pause) {
        dSpaceCollide (space,0,&nearCallback);
        dWorldStep (world,0.01);

        // remove all contact joints
        dJointGroupEmpty (contactgroup);
    }

    const dReal *pos = dBodyGetPosition(body[0]);
    printf("%f %f %f\n", pos[0], pos[1], pos[2]);

    dsSetTexture (DS_WOOD);
    dsSetColor (0,1,1);	dsDrawSphere (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sphere_radius);
    dsSetColor (1,0,0);	dsDrawCapsule (dBodyGetPosition(body[1]),dBodyGetRotation(body[1]),capsule_l, capsule_r);
}

int main (int argc, char **argv)
{
    g_factory = new StandardPartsFactory();

    ODESimlator* sim = new ODESimlator();
    sim->createWorld();
    sim->setFn(start, simLoop, command);
    sim->startSimulation();

    /*
    // hinge
    joint[0] = dJointCreateHinge(world, 0);
    dJointAttach(joint[0], body[0], body[1]);
    dJointSetHingeAnchor(joint[0], sphere_pos[0], sphere_pos[1], sphere_pos[2]-sphere_radius);
    dJointSetHingeAxis(joint[0], 1, 0, 0);
    dJointSetHingeParam(joint[0], dParamLoStop, -0.25*M_PI);
    dJointSetHingeParam(joint[0], dParamHiStop, +0.25*M_PI);
    */

    // run simulation

    /*
    dGeomDestroy (sphere[0]);
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
    dWorldDestroy (world);
    */
    dCloseODE();

    return 0;

