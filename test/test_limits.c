// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ranges of base.h at their edges. Each bound is taken at its edge,
// which a writer accepts, and one float past it, which the writer refuses,
// counting one misuse and changing nothing. Saturation (a body's velocity
// and accumulated load) and the clamp of mass data summed from shapes are
// checked at their limits too, and every world built here must restore its
// own snapshot.

#include "helpers.h"
#include "test_harness.h"

#include "maul2d/maul2d.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static float Past(float edge)
{
    return nextafterf(edge, edge < 0.0f ? -INFINITY : INFINITY);
}

static uint64_t Misuse(const m2World* world)
{
    return m2World_GetCounters(world).misuse;
}

// The edge lands and reads back; one float past it is refused, counted
// once, and leaves the edge in place.
typedef struct Channel
{
    m2World* world;
    void (*set)(const struct Channel* c, float value);
    float (*get)(const struct Channel* c);
    m2BodyId body;
    m2ShapeId shape;
    m2JointId joint;
    m2ParticleId particle;
} Channel;

static void ExpectEdge(const Channel* c, float edge, const char* what)
{
    uint64_t misuse = Misuse(c->world);
    c->set(c, edge);
    bool landed = c->get(c) == edge && Misuse(c->world) == misuse;
    c->set(c, Past(edge));
    bool refused = c->get(c) == edge && Misuse(c->world) == misuse + 1;
    if (!landed || !refused)
    {
        printf("  edge %g of %s: landed %d, refused %d\n", (double)edge, what, landed, refused);
    }
    CHECK(landed && refused, "an edge lands and one float past it is refused and counted");
}

static void SetLinearVelocity(const Channel* c, float v)
{
    m2Body_SetLinearVelocity(c->world, c->body, (m2Vec2){v, 0.0f});
}
static float GetLinearVelocity(const Channel* c)
{
    return m2Body_GetLinearVelocity(c->world, c->body).x;
}
static void SetAngularVelocity(const Channel* c, float v)
{
    m2Body_SetAngularVelocity(c->world, c->body, v);
}
static float GetAngularVelocity(const Channel* c)
{
    return m2Body_GetAngularVelocity(c->world, c->body);
}
static void SetGravityScale(const Channel* c, float v)
{
    m2Body_SetGravityScale(c->world, c->body, v);
}
static float GetGravityScale(const Channel* c)
{
    return m2Body_GetGravityScale(c->world, c->body);
}
static void SetLinearDamping(const Channel* c, float v)
{
    m2Body_SetLinearDamping(c->world, c->body, v);
}
static float GetLinearDamping(const Channel* c)
{
    return m2Body_GetLinearDamping(c->world, c->body);
}
static void SetAngularDamping(const Channel* c, float v)
{
    m2Body_SetAngularDamping(c->world, c->body, v);
}
static float GetAngularDamping(const Channel* c)
{
    return m2Body_GetAngularDamping(c->world, c->body);
}
static void SetFriction(const Channel* c, float v)
{
    m2Shape_SetFriction(c->world, c->shape, v);
}
static float GetFriction(const Channel* c)
{
    return m2Shape_GetFriction(c->world, c->shape);
}
static void SetDensity(const Channel* c, float v)
{
    m2Shape_SetDensity(c->world, c->shape, v);
}
static float GetDensity(const Channel* c)
{
    return m2Shape_GetDensity(c->world, c->shape);
}
static void SetTangentSpeed(const Channel* c, float v)
{
    m2Shape_SetTangentSpeed(c->world, c->shape, v);
}
static float GetTangentSpeed(const Channel* c)
{
    return m2Shape_GetTangentSpeed(c->world, c->shape);
}
static void SetGravity(const Channel* c, float v)
{
    m2World_SetGravity(c->world, (m2Vec2){0.0f, v});
}
static float GetGravity(const Channel* c)
{
    return m2World_GetGravity(c->world).y;
}
static void SetWindSpeed(const Channel* c, float v)
{
    m2World_SetWind(c->world, (m2Vec2){v, 0.0f}, 1.0f);
}
static float GetWindSpeed(const Channel* c)
{
    m2Vec2 wind = {0.0f, 0.0f};
    m2World_GetWind(c->world, &wind, NULL);
    return wind.x;
}
static void SetWindDrag(const Channel* c, float v)
{
    m2World_SetWind(c->world, (m2Vec2){1.0f, 0.0f}, v);
}
static float GetWindDrag(const Channel* c)
{
    float drag = 0.0f;
    m2World_GetWind(c->world, NULL, &drag);
    return drag;
}
static void SetMotorSpeed(const Channel* c, float v)
{
    m2Joint_SetMotorSpeed(c->world, c->joint, v);
}
static float GetMotorSpeed(const Channel* c)
{
    return m2Joint_GetMotorSpeed(c->world, c->joint);
}
static void SetMaxMotor(const Channel* c, float v)
{
    m2Joint_SetMaxMotor(c->world, c->joint, v);
}
static float GetMaxMotor(const Channel* c)
{
    return m2Joint_GetMaxMotor(c->world, c->joint);
}
static void SetSpringHertz(const Channel* c, float v)
{
    m2Joint_SetSpringHertz(c->world, c->joint, v);
}
static float GetSpringHertz(const Channel* c)
{
    return m2Joint_GetSpringHertz(c->world, c->joint);
}
static void SetSpringDampingRatio(const Channel* c, float v)
{
    m2Joint_SetSpringDampingRatio(c->world, c->joint, v);
}
static float GetSpringDampingRatio(const Channel* c)
{
    return m2Joint_GetSpringDampingRatio(c->world, c->joint);
}
static void SetUpperLimit(const Channel* c, float v)
{
    m2Joint_SetLimits(c->world, c->joint, 0.0f, v);
}
static float GetUpperLimit(const Channel* c)
{
    float lower = 0.0f;
    float upper = 0.0f;
    m2Joint_GetLimits(c->world, c->joint, &lower, &upper);
    return upper;
}
static void SetLowerLimit(const Channel* c, float v)
{
    m2Joint_SetLimits(c->world, c->joint, v, 0.0f);
}
static float GetLowerLimit(const Channel* c)
{
    float lower = 0.0f;
    float upper = 0.0f;
    m2Joint_GetLimits(c->world, c->joint, &lower, &upper);
    return lower;
}
static void SetLength(const Channel* c, float v)
{
    m2DistanceJoint_SetLength(c->world, c->joint, v);
}
static float GetLength(const Channel* c)
{
    return m2Joint_GetLength(c->world, c->joint);
}
static void SetGearRatio(const Channel* c, float v)
{
    m2GearJoint_SetRatio(c->world, c->joint, v);
}
static float GetGearRatio(const Channel* c)
{
    return m2GearJoint_GetRatio(c->world, c->joint);
}
static void SetParticleVelocity(const Channel* c, float v)
{
    m2Particle_SetVelocity(c->world, c->particle, (m2Vec2){0.0f, v});
}
static float GetParticleVelocity(const Channel* c)
{
    return m2Particle_GetVelocity(c->world, c->particle).y;
}

// A world with particles and a fluid volume, so every part has its
// writers; a box falls on the ground.
static m2World* LimitsWorld(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.particleCapacity = 16;
    def.fluidVolumeCapacity = 2;
    return NewWorld(&def);
}

static m2BodyId Box(m2World* world, m2BodyType type, float x)
{
    m2BodyDef def = m2DefaultBodyDef();
    def.type = type;
    def.position = (m2Pos2){(double)x, 2.0};
    m2BodyId body = NewBody(world, &def);
    m2ShapeDef shape = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    (void)NewPolygonShape(world, body, &shape, &box);
    return body;
}

static void ExpectRestores(m2World* world)
{
    int32_t size = m2World_SnapshotSize(world);
    uint8_t* bytes = (uint8_t*)malloc((size_t)size);
    CHECK(bytes != NULL, "snapshot buffer");
    uint64_t hash = m2World_Hash(world);
    CHECK(Snapshot(world, bytes, size) == size && Restore(world, bytes, size) &&
              m2World_Hash(world) == hash,
          "a world at its edges restores its own snapshot");
    free(bytes);
}

static void TestBodyAndWorldEdges(void)
{
    m2World* world = LimitsWorld();
    Channel c = {world,
                 NULL,
                 NULL,
                 Box(world, m2_dynamicBody, 0.0f),
                 m2_nullShapeId,
                 m2_nullJointId,
                 m2_nullParticleId};
    struct
    {
        void (*set)(const Channel* c, float value);
        float (*get)(const Channel* c);
        float edge;
        const char* what;
    } bodies[] = {
        {SetLinearVelocity, GetLinearVelocity, M2_MAX_SPEED, "linear velocity"},
        {SetLinearVelocity, GetLinearVelocity, -M2_MAX_SPEED, "linear velocity"},
        {SetAngularVelocity, GetAngularVelocity, -M2_MAX_SPEED, "angular velocity"},
        {SetGravityScale, GetGravityScale, M2_MAX_GRAVITY_SCALE, "gravity scale"},
        {SetGravityScale, GetGravityScale, -M2_MAX_GRAVITY_SCALE, "gravity scale"},
        {SetLinearDamping, GetLinearDamping, M2_MAX_RATE, "linear damping"},
        {SetAngularDamping, GetAngularDamping, M2_MAX_RATE, "angular damping"},
        {SetGravity, GetGravity, -M2_MAX_ACCELERATION, "gravity"},
        {SetWindSpeed, GetWindSpeed, M2_MAX_SPEED, "wind speed"},
        {SetWindDrag, GetWindDrag, M2_MAX_RATE, "wind drag"},
    };
    for (size_t i = 0; i < sizeof(bodies) / sizeof(bodies[0]); ++i)
    {
        c.set = bodies[i].set;
        c.get = bodies[i].get;
        ExpectEdge(&c, bodies[i].edge, bodies[i].what);
    }
    ExpectRestores(world);
    m2DestroyWorld(world);
}

static void TestShapeEdges(void)
{
    m2World* world = LimitsWorld();
    m2BodyId body = Box(world, m2_dynamicBody, 0.0f);
    m2ShapeId shape = m2_nullShapeId;
    CHECK(m2Body_GetShapes(world, body, &shape, 1) == 1, "the box's shape");
    Channel c = {world, NULL, NULL, body, shape, m2_nullJointId, m2_nullParticleId};
    c.set = SetFriction;
    c.get = GetFriction;
    ExpectEdge(&c, M2_MAX_COEFFICIENT, "friction");
    c.set = SetDensity;
    c.get = GetDensity;
    ExpectEdge(&c, M2_MAX_DENSITY, "density");
    c.set = SetTangentSpeed;
    c.get = GetTangentSpeed;
    ExpectEdge(&c, -M2_MAX_SPEED, "tangent speed");

    // The def takes the same ranges as the setters.
    m2ShapeDef def = m2DefaultShapeDef();
    m2Circle circle = {{0.0f, 0.0f}, 0.25f};
    uint64_t misuse = Misuse(world);
    def.density = Past(M2_MAX_DENSITY);
    m2ShapeId refused = {7, 7};
    CHECK(m2CreateCircleShape(world, body, &def, &circle, &refused) == m2_errorInvalid &&
              refused.index1 == 0 && Misuse(world) == misuse + 1,
          "a shape def past the density bound is refused and counted");
    def.density = M2_MAX_DENSITY;
    def.friction = M2_MAX_COEFFICIENT;
    def.tangentSpeed = M2_MAX_SPEED;
    CHECK(NewCircleShape(world, body, &def, &circle).index1 != 0,
          "a shape def at its edges is accepted");
    ExpectRestores(world);
    m2DestroyWorld(world);
}

static void TestJointEdges(void)
{
    m2World* world = LimitsWorld();
    m2BodyId a = Box(world, m2_dynamicBody, 0.0f);
    m2BodyId b = Box(world, m2_dynamicBody, 2.0f);
    Channel c = {world, NULL, NULL, a, m2_nullShapeId, m2_nullJointId, m2_nullParticleId};

    m2RevoluteJointDef revolute = m2DefaultRevoluteJointDef();
    revolute.bodyIdA = a;
    revolute.bodyIdB = b;
    c.joint = NewRevoluteJoint(world, &revolute);
    c.set = SetMotorSpeed;
    c.get = GetMotorSpeed;
    ExpectEdge(&c, -M2_MAX_SPEED, "motor speed");
    c.set = SetMaxMotor;
    c.get = GetMaxMotor;
    ExpectEdge(&c, M2_MAX_FORCE, "motor budget");
    c.set = SetSpringHertz;
    c.get = GetSpringHertz;
    ExpectEdge(&c, M2_MAX_RATE, "spring hertz");
    c.set = SetSpringDampingRatio;
    c.get = GetSpringDampingRatio;
    ExpectEdge(&c, M2_MAX_COEFFICIENT, "spring damping ratio");
    c.set = SetUpperLimit;
    c.get = GetUpperLimit;
    ExpectEdge(&c, M2_MAX_ANGLE, "revolute upper angle");

    m2PrismaticJointDef prismatic = m2DefaultPrismaticJointDef();
    prismatic.bodyIdA = a;
    prismatic.bodyIdB = b;
    c.joint = NewPrismaticJoint(world, &prismatic);
    c.set = SetLowerLimit;
    c.get = GetLowerLimit;
    ExpectEdge(&c, -(float)M2_JOINT_REACH, "prismatic lower translation");

    m2DistanceJointDef distance = m2DefaultDistanceJointDef();
    distance.bodyIdA = a;
    distance.bodyIdB = b;
    c.joint = NewDistanceJoint(world, &distance);
    c.set = SetLength;
    c.get = GetLength;
    ExpectEdge(&c, (float)M2_JOINT_REACH, "distance length");

    m2GearJointDef gear = m2DefaultGearJointDef();
    gear.bodyIdA = a;
    gear.bodyIdB = b;
    c.joint = NewGearJoint(world, &gear);
    c.set = SetGearRatio;
    c.get = GetGearRatio;
    ExpectEdge(&c, M2_MAX_RATIO, "gear ratio");
    ExpectEdge(&c, -M2_MAX_RATIO, "gear ratio");
    // Below the smallest ratio is one float toward zero.
    uint64_t misuse = Misuse(world);
    m2GearJoint_SetRatio(world, c.joint, M2_MIN_RATIO);
    m2GearJoint_SetRatio(world, c.joint, nextafterf(M2_MIN_RATIO, 0.0f));
    CHECK(m2GearJoint_GetRatio(world, c.joint) == M2_MIN_RATIO && Misuse(world) == misuse + 1,
          "a gear ratio below the smallest is refused and counted");

    m2RatchetJointDef ratchet = m2DefaultRatchetJointDef();
    ratchet.bodyIdA = a;
    ratchet.bodyIdB = b;
    ratchet.ratchet = -M2_MIN_RATCHET_TOOTH;
    CHECK(NewRatchetJoint(world, &ratchet).index1 != 0, "the smallest ratchet tooth is accepted");
    ratchet.ratchet = nextafterf(M2_MIN_RATCHET_TOOTH, 0.0f);
    misuse = Misuse(world);
    m2JointId refused = {7, 7};
    CHECK(m2CreateRatchetJoint(world, &ratchet, &refused) == m2_errorInvalid &&
              refused.index1 == 0 && Misuse(world) == misuse + 1,
          "a ratchet tooth below the smallest is refused and counted");

    m2WeldJointDef weld = m2DefaultWeldJointDef();
    weld.bodyIdA = a;
    weld.bodyIdB = b;
    weld.angularHertz = Past(M2_MAX_RATE);
    misuse = Misuse(world);
    CHECK(m2CreateWeldJoint(world, &weld, &refused) == m2_errorInvalid && refused.index1 == 0 &&
              Misuse(world) == misuse + 1,
          "a weld def past the rate bound is refused and counted");
    ExpectRestores(world);
    m2DestroyWorld(world);
}

static void TestParticleAndVolumeEdges(void)
{
    m2World* world = LimitsWorld();
    Channel c = {
        world, NULL, NULL, m2_nullBodyId, m2_nullShapeId, m2_nullJointId, m2_nullParticleId};
    c.particle = EmitParticle(world, (m2Pos2){0.0, 5.0}, (m2Vec2){0.0f, 0.0f}, 0);
    c.set = SetParticleVelocity;
    c.get = GetParticleVelocity;
    ExpectEdge(&c, -M2_MAX_SPEED, "particle velocity");

    m2FluidVolumeDef volume = m2DefaultFluidVolumeDef();
    volume.regionLower = (m2Pos2){-10.0, -10.0};
    volume.regionUpper = (m2Pos2){10.0, 0.0};
    volume.density = M2_MAX_DENSITY;
    volume.linearDrag = M2_MAX_RATE;
    volume.flow = (m2Vec2){M2_MAX_SPEED, -M2_MAX_SPEED};
    CHECK(NewFluidVolume(world, &volume).index1 != 0, "a fluid volume at its edges is accepted");
    volume.flow.y = Past(-M2_MAX_SPEED);
    uint64_t misuse = Misuse(world);
    m2FluidVolumeId refused = {7, 7};
    CHECK(m2CreateFluidVolume(world, &volume, &refused) == m2_errorInvalid && refused.index1 == 0 &&
              Misuse(world) == misuse + 1,
          "a fluid volume past the speed bound is refused and counted");

    // The particles' mass must be a mass a body could have.
    m2WorldDef def = m2DefaultWorldDef();
    def.particleCapacity = 4;
    def.particleRadius = 0.02f;
    def.particleDensity = 1.0e-6f; // 9e-10 kg, below M2_MIN_MASS
    m2World* light = NULL;
    CHECK(m2CreateWorld(&def, &light) == m2_errorInvalid && light == NULL,
          "particles lighter than M2_MIN_MASS are refused");
    if (light != NULL)
    {
        m2DestroyWorld(light);
    }
    ExpectRestores(world);
    m2DestroyWorld(world);
}

// m2Body_SetMassData takes mass and inertia about the center within their
// ranges; mass summed from shapes is clamped into them and reads back as
// kept.
static void TestMassEdges(void)
{
    m2World* world = LimitsWorld();
    m2BodyId body = Box(world, m2_dynamicBody, 0.0f);
    struct
    {
        float mass;
        float inertia;
        bool accepted;
    } cases[] = {
        {M2_MIN_MASS, M2_MIN_INERTIA, true}, {M2_MAX_MASS, M2_MAX_INERTIA, true},
        {M2_MIN_MASS, 0.0f, true},           {nextafterf(M2_MIN_MASS, 0.0f), 1.0f, false},
        {Past(M2_MAX_MASS), 1.0f, false},    {1.0f, nextafterf(M2_MIN_INERTIA, 0.0f), false},
        {1.0f, Past(M2_MAX_INERTIA), false},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        m2MassData data = {cases[i].mass, {0.0f, 0.0f}, cases[i].inertia};
        uint64_t misuse = Misuse(world);
        m2Body_SetMassData(world, body, data);
        bool counted = Misuse(world) == misuse + (cases[i].accepted ? 0 : 1);
        if (cases[i].accepted)
        {
            m2MassData back = m2Body_GetMassData(world, body);
            counted = counted && back.mass == 1.0f / (1.0f / cases[i].mass);
            ExpectRestores(world);
        }
        CHECK(counted, "mass data at its edges lands; one float past is refused and counted");
    }

    // A speck of foam and a slab of lead past every realistic size: the
    // first is clamped up to the smallest mass and inertia, the second
    // down to the largest.
    m2ShapeDef foam = m2DefaultShapeDef();
    foam.density = 1.0e-6f;
    m2Circle speck = {{0.0f, 0.0f}, 0.005f};
    m2BodyDef bodyDef = m2DefaultBodyDef();
    bodyDef.type = m2_dynamicBody;
    m2BodyId small = NewBody(world, &bodyDef);
    (void)NewCircleShape(world, small, &foam, &speck);
    m2MassData kept = m2Body_GetMassData(world, small);
    CHECK(kept.mass == 1.0f / (1.0f / M2_MIN_MASS) &&
              kept.centralInertia == 1.0f / (1.0f / M2_MIN_INERTIA),
          "a speck's mass and inertia are clamped up to the smallest");

    m2ShapeDef lead = m2DefaultShapeDef();
    lead.density = M2_MAX_DENSITY;
    m2Circle slab = {{0.0f, 0.0f}, M2_HUGE};
    bodyDef.position = (m2Pos2){1.0e7, 0.0};
    m2BodyId large = NewBody(world, &bodyDef);
    (void)NewCircleShape(world, large, &lead, &slab);
    kept = m2Body_GetMassData(world, large);
    CHECK(kept.mass == 1.0f / (1.0f / M2_MAX_MASS) &&
              kept.centralInertia == 1.0f / (1.0f / M2_MAX_INERTIA),
          "a giant's mass and inertia are clamped down to the largest");
    ExpectRestores(world);
    m2DestroyWorld(world);
}

// Impulses and forces at their bounds: one float past is refused; what a
// body accumulates saturates.
static void TestLoadEdges(void)
{
    m2World* world = LimitsWorld();
    m2World_SetGravity(world, (m2Vec2){0.0f, 0.0f});
    m2BodyId body = Box(world, m2_dynamicBody, 0.0f);
    uint64_t misuse = Misuse(world);
    m2Body_ApplyLinearImpulse(world, body, (m2Vec2){Past(M2_IMPULSE_LIMIT), 0.0f});
    m2Body_ApplyAngularImpulse(world, body, Past(-M2_IMPULSE_LIMIT));
    m2Body_ApplyForce(world, body, (m2Vec2){0.0f, Past(M2_MAX_FORCE)});
    m2Body_ApplyTorque(world, body, Past(M2_MAX_FORCE));
    CHECK(Misuse(world) == misuse + 4 && m2Body_GetLinearVelocity(world, body).x == 0.0f &&
              m2Body_GetAngularVelocity(world, body) == 0.0f,
          "loads one float past their bounds are refused and counted");

    m2Body_ApplyLinearImpulse(world, body, (m2Vec2){M2_IMPULSE_LIMIT, -M2_IMPULSE_LIMIT});
    m2Body_ApplyAngularImpulse(world, body, M2_IMPULSE_LIMIT);
    m2Vec2 v = m2Body_GetLinearVelocity(world, body);
    CHECK(Misuse(world) == misuse + 4 && v.x == M2_MAX_SPEED && v.y == -M2_MAX_SPEED &&
              m2Body_GetAngularVelocity(world, body) == M2_MAX_SPEED,
          "the largest impulses land and the velocity saturates at M2_MAX_SPEED");

    // Twice the largest force is the largest force: two twins step to the
    // same bits.
    m2BodyId twin = Box(world, m2_dynamicBody, 10.0f);
    m2Body_SetLinearVelocity(world, body, (m2Vec2){0.0f, 0.0f});
    m2Body_SetAngularVelocity(world, body, 0.0f);
    m2Body_ApplyForce(world, body, (m2Vec2){M2_MAX_FORCE, 0.0f});
    m2Body_ApplyForce(world, body, (m2Vec2){M2_MAX_FORCE, 0.0f});
    m2Body_ApplyForce(world, twin, (m2Vec2){M2_MAX_FORCE, 0.0f});
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_GetLinearVelocity(world, body).x == m2Body_GetLinearVelocity(world, twin).x,
          "an accumulated force saturates at M2_MAX_FORCE");
    ExpectRestores(world);
    m2DestroyWorld(world);
}

// An explosion's impulse, radius, falloff and position at their edges: a
// blast at its edge lands, one float past any of them, or a NaN, is
// refused, counted and changes nothing.
static void TestExplosionEdges(void)
{
    m2World* world = LimitsWorld();
    m2World_SetGravity(world, (m2Vec2){0.0f, 0.0f});
    m2BodyId body = Box(world, m2_dynamicBody, 0.0f);
    m2ExplosionDef edge = m2DefaultExplosionDef();
    edge.position = (m2Pos2){0.0, 0.0};
    edge.radius = M2_HUGE;
    edge.falloff = M2_HUGE;
    edge.impulse = -M2_IMPULSE_LIMIT;
    uint64_t misuse = Misuse(world);
    m2World_Explode(world, &edge);
    m2Vec2 v = m2Body_GetLinearVelocity(world, body);
    CHECK(Misuse(world) == misuse && fabsf(v.y) == M2_MAX_SPEED,
          "a blast at its edges lands and saturates the body's velocity");

    m2Body_SetLinearVelocity(world, body, (m2Vec2){0.0f, 0.0f});
    for (int32_t field = 0; field < 7; ++field)
    {
        m2ExplosionDef bad = edge;
        switch (field)
        {
        case 0:
            bad.impulse = Past(-M2_IMPULSE_LIMIT);
            break;
        case 1:
            bad.impulse = NAN;
            break;
        case 2:
            bad.radius = Past(M2_HUGE);
            break;
        case 3:
            bad.falloff = Past(M2_HUGE);
            break;
        case 4:
            bad.falloff = 0.0f;
            break;
        case 5:
            bad.position.x = (double)NAN;
            break;
        default:
            bad.position.y = (double)INFINITY;
            break;
        }
        misuse = Misuse(world);
        m2World_Explode(world, &bad);
        v = m2Body_GetLinearVelocity(world, body);
        CHECK(Misuse(world) == misuse + 1 && v.x == 0.0f && v.y == 0.0f,
              "a blast one past an edge is refused, counted and changes nothing");
    }
    ExpectRestores(world);
    m2DestroyWorld(world);
}

// A force or impulse at a world point farther than M2_JOINT_REACH acts
// with its arm clamped to the reach: the same as at the reach, and
// finite, where an unclamped 1e300 m arm would be infinite.
static void TestFarPointsClampTheArm(void)
{
    m2World* world = LimitsWorld();
    m2World_SetGravity(world, (m2Vec2){0.0f, 0.0f});
    m2BodyId far = Box(world, m2_dynamicBody, 0.0f);
    m2BodyId reach = Box(world, m2_dynamicBody, 10.0f);
    m2Pos2 centerFar = m2Body_GetWorldCenterOfMass(world, far);
    m2Pos2 centerReach = m2Body_GetWorldCenterOfMass(world, reach);
    m2Vec2 tiny = {0.0f, 1.0e-12f};
    m2Body_ApplyLinearImpulseAtPoint(world, far, tiny, (m2Pos2){1.0e300, centerFar.y});
    m2Body_ApplyLinearImpulseAtPoint(world, reach, tiny,
                                     (m2Pos2){centerReach.x + M2_JOINT_REACH, centerReach.y});
    float wFar = m2Body_GetAngularVelocity(world, far);
    CHECK(isfinite(wFar) && wFar > 0.0f && wFar == m2Body_GetAngularVelocity(world, reach),
          "an impulse at a far point turns the body as one at the reach does");

    m2Body_ApplyForceAtPoint(world, far, tiny, (m2Pos2){-1.0e300, centerFar.y});
    m2Body_ApplyForceAtPoint(world, reach, tiny,
                             (m2Pos2){centerReach.x - M2_JOINT_REACH, centerReach.y});
    m2World_Step(world, 1.0f / 60.0f, 4);
    wFar = m2Body_GetAngularVelocity(world, far);
    CHECK(isfinite(wFar) && wFar == m2Body_GetAngularVelocity(world, reach),
          "a force at a far point turns the body as one at the reach does");
    ExpectRestores(world);
    m2DestroyWorld(world);
}

// The step and a target's time step at their edges.
static void TestTimeStepEdges(void)
{
    m2World* world = LimitsWorld();
    m2BodyId lift = Box(world, m2_kinematicBody, 0.0f);
    float edges[] = {M2_MIN_TIME_STEP, M2_MAX_TIME_STEP};
    float past[] = {nextafterf(M2_MIN_TIME_STEP, 0.0f), Past(M2_MAX_TIME_STEP)};
    for (int32_t i = 0; i < 2; ++i)
    {
        uint64_t steps = m2World_GetStepCount(world);
        uint64_t misuse = Misuse(world);
        m2World_Step(world, edges[i], 1);
        m2World_Step(world, past[i], 1);
        CHECK(m2World_GetStepCount(world) == steps + 1 && Misuse(world) == misuse + 1,
              "a step at its edge runs and one float past it is refused and counted");

        m2Body_SetTargetTransform(world, lift, (m2Pos2){0.0, 2.0}, m2MakeRot(0.0f), edges[i]);
        m2Body_SetTargetTransform(world, lift, (m2Pos2){0.0, 2.0}, m2MakeRot(0.0f), past[i]);
        CHECK(Misuse(world) == misuse + 2, "a target's time step past its edge is refused");
    }
    // A target too far to reach within M2_MAX_SPEED in one step is refused
    // whole: neither velocity changes.
    m2Body_SetAngularVelocity(world, lift, 1.0f);
    uint64_t misuse = Misuse(world);
    m2Body_SetTargetTransform(world, lift, (m2Pos2){1.0e6, 2.0}, m2MakeRot(1.0f), 1.0f);
    CHECK(Misuse(world) == misuse + 1 && m2Body_GetAngularVelocity(world, lift) == 1.0f,
          "a target past M2_MAX_SPEED is refused before either velocity is set");
    ExpectRestores(world);
    m2DestroyWorld(world);
}

int main(void)
{
    TestBodyAndWorldEdges();
    TestShapeEdges();
    TestJointEdges();
    TestParticleAndVolumeEdges();
    TestMassEdges();
    TestLoadEdges();
    TestTimeStepEdges();
    TestExplosionEdges();
    TestFarPointsClampTheArm();
    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
