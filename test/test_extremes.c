// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Scenes built from the edges of base.h's ranges: the lightest and
// heaviest bodies, the longest levers, the stiffest springs, the fastest
// speeds and the strongest loads the API accepts. Every step must keep
// the world valid with every velocity within M2_MAX_SPEED, and the world
// must restore its own snapshot after every step: what the step saturates
// stays within the ranges a restore enforces, to the last float. Without the solver's
// saturation these scenes reach infinity and NaN. On glibc, in builds
// with assertions, they run with overflow, invalid operations and
// division by zero trapping, so a single product past the float range
// anywhere in a step stops the test there. Optimized builds may evaluate
// a guarded division ahead of its guard (they compile without trapping
// math), so they check only the results.

#define _GNU_SOURCE // feenableexcept

#include "helpers.h"
#include "test_harness.h"

#include "maul2d/maul2d.h"

#include <fenv.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define STEPS 60

typedef struct Scene
{
    m2World* world;
    m2BodyId bodies[8];
    int32_t bodyCount;
    m2JointId joints[16];
    int32_t jointCount;
    float steadyDt; // 0: alternate the smallest and largest steps
} Scene;

static m2JointId Keep(Scene* s, m2JointId joint)
{
    s->joints[s->jointCount++] = joint;
    return joint;
}

// Every impulse a joint or contact keeps is within M2_IMPULSE_LIMIT. A
// joint's reaction is its impulses over the last substep, summed over at
// most four rows.
static bool ImpulsesWithinLimit(const Scene* s, float invH)
{
    for (int32_t j = 0; j < s->jointCount; ++j)
    {
        if (m2Joint_GetReactionForce(s->world, s->joints[j]) > 4.0f * M2_IMPULSE_LIMIT * invH ||
            m2Joint_GetReactionTorque(s->world, s->joints[j]) > 4.0f * M2_IMPULSE_LIMIT * invH)
        {
            return false;
        }
    }
    m2ContactData data[16];
    int32_t count = m2World_GetContactData(s->world, data, 16);
    for (int32_t c = 0; c < count; ++c)
    {
        for (int32_t k = 0; k < data[c].pointCount; ++k)
        {
            if (!(data[c].normalImpulses[k] <= M2_IMPULSE_LIMIT) ||
                !(fabsf(data[c].tangentImpulses[k]) <= M2_IMPULSE_LIMIT))
            {
                return false;
            }
        }
    }
    return true;
}

// The world's own snapshot restores onto the same hash.
static bool RoundTrips(m2World* world)
{
    int32_t size = m2World_SnapshotSize(world);
    void* bytes = malloc((size_t)size);
    if (bytes == NULL)
    {
        return false;
    }
    uint64_t hash = m2World_Hash(world);
    bool restores = Snapshot(world, bytes, size) == size && Restore(world, bytes, size) &&
                    m2World_Hash(world) == hash;
    free(bytes);
    return restores;
}

static m2World* ExtremeWorld(bool particles)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.gravity = (m2Vec2){M2_MAX_ACCELERATION, -M2_MAX_ACCELERATION};
    def.fluidVolumeCapacity = 2;
    if (particles)
    {
        def.particleCapacity = 64;
        def.particleDensity = M2_MAX_DENSITY;
        def.particleGravityScale = M2_MAX_GRAVITY_SCALE;
        def.particlePressureStrength = M2_MAX_COEFFICIENT;
        def.particleDampingStrength = M2_MAX_COEFFICIENT;
        def.particleViscousStrength = M2_MAX_COEFFICIENT;
        def.particlePowderStrength = M2_MAX_COEFFICIENT;
        def.particleSpringStrength = M2_MAX_COEFFICIENT;
        def.particleElasticStrength = M2_MAX_COEFFICIENT;
        def.particleCohesionStrength = M2_MAX_COEFFICIENT;
        def.particleNearPressureStrength = M2_MAX_COEFFICIENT;
    }
    return NewWorld(&def);
}

static m2BodyId Add(Scene* s, m2BodyType type, m2Pos2 position, m2Vec2 velocity, float spin)
{
    m2BodyDef def = m2DefaultBodyDef();
    def.type = type;
    def.position = position;
    if (type != m2_staticBody)
    {
        def.linearVelocity = velocity;
        def.angularVelocity = spin;
        def.gravityScale = M2_MAX_GRAVITY_SCALE;
    }
    m2BodyId body = NewBody(s->world, &def);
    s->bodies[s->bodyCount++] = body;
    return body;
}

// The lightest body: a 5 mm disc of foam, clamped to M2_MIN_MASS and
// M2_MIN_INERTIA.
static m2BodyId Speck(Scene* s, m2Pos2 position, m2Vec2 velocity)
{
    m2BodyId body = Add(s, m2_dynamicBody, position, velocity, -M2_MAX_SPEED);
    m2ShapeDef shape = m2DefaultShapeDef();
    shape.density = 1.0e-9f;
    shape.friction = M2_MAX_COEFFICIENT;
    shape.restitution = 1.0f;
    shape.tangentSpeed = M2_MAX_SPEED;
    m2Circle circle = {{0.0f, 0.0f}, 0.005f};
    (void)NewCircleShape(s->world, body, &shape, &circle);
    return body;
}

// The heaviest: a kilometer block at the largest density, clamped to
// M2_MAX_MASS and M2_MAX_INERTIA.
static m2BodyId Giant(Scene* s, m2Pos2 position, m2Vec2 velocity)
{
    m2BodyId body = Add(s, m2_dynamicBody, position, velocity, M2_MAX_SPEED);
    m2ShapeDef shape = m2DefaultShapeDef();
    shape.density = M2_MAX_DENSITY;
    shape.friction = M2_MAX_COEFFICIENT;
    shape.tangentSpeed = -M2_MAX_SPEED;
    m2Polygon box = m2MakeBox(500.0f, 500.0f);
    (void)NewPolygonShape(s->world, body, &shape, &box);
    return body;
}

static void ExpectFinite(Scene* s, const char* what)
{
    bool valid = true;
    for (int32_t step = 0; step < STEPS && valid; ++step)
    {
        for (int32_t i = 0; i < s->bodyCount; ++i)
        {
            if (m2Body_GetType(s->world, s->bodies[i]) == m2_dynamicBody)
            {
                m2Body_ApplyForce(s->world, s->bodies[i], (m2Vec2){M2_MAX_FORCE, -M2_MAX_FORCE});
                m2Body_ApplyTorque(s->world, s->bodies[i], M2_MAX_FORCE);
            }
        }
        float dt = step % 2 == 0 ? M2_MIN_TIME_STEP : M2_MAX_TIME_STEP;
        int32_t substeps = step % 3 == 0 ? M2_MAX_SUBSTEPS : 1;
        if (s->steadyDt > 0.0f)
        {
            dt = s->steadyDt;
            substeps = 4;
        }
        m2World_Step(s->world, dt, substeps);
        valid = m2World_Validate(s->world) && ImpulsesWithinLimit(s, 1.0f / (dt / (float)substeps));
        if (valid && !RoundTrips(s->world))
        {
            printf("  %s: its own snapshot is refused after step %d\n", what, step);
            valid = false;
        }
        for (int32_t i = 0; i < s->bodyCount && valid; ++i)
        {
            m2Vec2 v = m2Body_GetLinearVelocity(s->world, s->bodies[i]);
            float w = m2Body_GetAngularVelocity(s->world, s->bodies[i]);
            valid = fabsf(v.x) <= M2_MAX_SPEED && fabsf(v.y) <= M2_MAX_SPEED &&
                    fabsf(w) <= M2_MAX_SPEED;
        }
        if (!valid)
        {
            printf("  %s: invalid after step %d\n", what, step);
        }
    }
    CHECK(valid, "an extreme scene stays valid, within M2_MAX_SPEED, and restores its own "
                 "snapshot after every step");
    m2DestroyWorld(s->world);
}

// A giant falls at the largest speed onto specks resting on the ground.
static void TestGiantOnSpecks(void)
{
    Scene s = {ExtremeWorld(false), {{0}}, 0, {{0}}, 0, 0.0f};
    m2BodyId ground = Add(&s, m2_staticBody, (m2Pos2){0.0, 0.0}, (m2Vec2){0.0f, 0.0f}, 0.0f);
    m2ShapeDef shape = m2DefaultShapeDef();
    m2Polygon floor = m2MakeBox(M2_HUGE, 1.0f);
    (void)NewPolygonShape(s.world, ground, &shape, &floor);
    (void)Speck(&s, (m2Pos2){0.0, 1.005}, (m2Vec2){0.0f, 0.0f});
    (void)Speck(&s, (m2Pos2){0.3, 1.005}, (m2Vec2){M2_MAX_SPEED, 0.0f});
    (void)Giant(&s, (m2Pos2){0.0, 502.0}, (m2Vec2){-M2_MAX_SPEED, -M2_MAX_SPEED});
    ExpectFinite(&s, "giant on specks");
}

// A giant resting on a thick floor under the strongest gravity, stepped
// steadily so it stays there: its contacts need impulses past any limit.
static void TestGiantResting(void)
{
    Scene s = {ExtremeWorld(false), {{0}}, 0, {{0}}, 0, 1.0f / 60.0f};
    m2World_SetGravity(s.world, (m2Vec2){0.0f, -M2_MAX_ACCELERATION});
    m2BodyId ground = Add(&s, m2_staticBody, (m2Pos2){0.0, -1000.0}, (m2Vec2){0.0f, 0.0f}, 0.0f);
    m2ShapeDef shape = m2DefaultShapeDef();
    m2Polygon floor = m2MakeBox(M2_HUGE, 1000.0f);
    (void)NewPolygonShape(s.world, ground, &shape, &floor);
    m2BodyId giant = Giant(&s, (m2Pos2){0.0, 500.0}, (m2Vec2){0.0f, 0.0f});
    m2Body_SetAngularVelocity(s.world, giant, 0.0f);
    ExpectFinite(&s, "giant resting");
}

// A giant hanging from the ground by a rope and held off it by a strut
// (a length range with a minimum only, whose maximum is open-ended), under
// the strongest gravity: the impulses the rows need pass any limit.
static void TestHangingGiant(void)
{
    Scene s = {ExtremeWorld(false), {{0}}, 0, {{0}}, 0, 0.0f};
    m2BodyId anchor = Add(&s, m2_staticBody, (m2Pos2){0.0, 0.0}, (m2Vec2){0.0f, 0.0f}, 0.0f);
    m2BodyId giant = Giant(&s, (m2Pos2){0.0, -2000.0}, (m2Vec2){0.0f, -M2_MAX_SPEED});
    m2DistanceJointDef rope = m2DefaultDistanceJointDef();
    rope.bodyIdA = anchor;
    rope.bodyIdB = giant;
    rope.length = 2000.0f;
    (void)Keep(&s, NewDistanceJoint(s.world, &rope));
    m2DistanceJointDef strut = m2DefaultDistanceJointDef();
    strut.bodyIdA = anchor;
    strut.bodyIdB = giant;
    strut.localAnchorB = (m2Vec2){M2_HUGE, M2_HUGE};
    strut.enableSpring = true;
    strut.minLength = 10.0f;
    (void)Keep(&s, NewDistanceJoint(s.world, &strut));
    ExpectFinite(&s, "hanging giant");
}

// Specks held by joints at the longest levers: a hinge at the far edge of
// a spinning giant, a slider and a rope a reach away, a weld, a wheel.
static void TestLongLevers(void)
{
    Scene s = {ExtremeWorld(false), {{0}}, 0, {{0}}, 0, 0.0f};
    m2BodyId giant = Giant(&s, (m2Pos2){0.0, 0.0}, (m2Vec2){M2_MAX_SPEED, 0.0f});
    m2BodyId edge = Speck(&s, (m2Pos2){(double)M2_HUGE, 0.0}, (m2Vec2){0.0f, M2_MAX_SPEED});
    m2BodyId beyond = Speck(&s, (m2Pos2){M2_JOINT_REACH * 4.0, 0.0}, (m2Vec2){0.0f, 0.0f});

    m2RevoluteJointDef hinge = m2DefaultRevoluteJointDef();
    hinge.bodyIdA = giant;
    hinge.bodyIdB = edge;
    hinge.localAnchorA = (m2Vec2){M2_HUGE, M2_HUGE};
    hinge.localAnchorB = (m2Vec2){-M2_HUGE, M2_HUGE};
    hinge.hertz = M2_MAX_RATE;
    hinge.springHertz = M2_MAX_RATE;
    hinge.springDampingRatio = M2_MAX_COEFFICIENT;
    hinge.enableMotor = true;
    hinge.motorSpeed = M2_MAX_SPEED;
    hinge.maxMotorTorque = M2_MAX_FORCE;
    hinge.enableLimit = true;
    hinge.lowerAngle = -M2_MAX_ANGLE;
    hinge.upperAngle = -M2_MAX_ANGLE;
    (void)Keep(&s, NewRevoluteJoint(s.world, &hinge));

    m2PrismaticJointDef slider = m2DefaultPrismaticJointDef();
    slider.bodyIdA = edge;
    slider.bodyIdB = beyond;
    slider.localAxisA = (m2Vec2){0.0f, 1.0f};
    slider.localAnchorA = (m2Vec2){M2_HUGE, -M2_HUGE};
    slider.enableLimit = true;
    slider.lowerTranslation = (float)M2_JOINT_REACH;
    slider.upperTranslation = (float)M2_JOINT_REACH;
    slider.enableMotor = true;
    slider.motorSpeed = -M2_MAX_SPEED;
    slider.maxMotorForce = M2_MAX_FORCE;
    (void)Keep(&s, NewPrismaticJoint(s.world, &slider));

    m2DistanceJointDef rope = m2DefaultDistanceJointDef();
    rope.bodyIdA = giant;
    rope.bodyIdB = beyond;
    rope.length = 0.005f;
    rope.hertz = M2_MAX_RATE;
    rope.dampingRatio = M2_MAX_COEFFICIENT;
    (void)Keep(&s, NewDistanceJoint(s.world, &rope));

    m2WheelJointDef wheel = m2DefaultWheelJointDef();
    wheel.bodyIdA = beyond;
    wheel.bodyIdB = giant;
    wheel.localAnchorB = (m2Vec2){-M2_HUGE, -M2_HUGE};
    wheel.hertz = M2_MAX_RATE;
    wheel.enableMotor = true;
    wheel.motorSpeed = M2_MAX_SPEED;
    wheel.maxMotorTorque = M2_MAX_FORCE;
    (void)Keep(&s, NewWheelJoint(s.world, &wheel));

    m2WeldJointDef weld = m2DefaultWeldJointDef();
    weld.bodyIdA = edge;
    weld.bodyIdB = giant;
    weld.localAnchorA = (m2Vec2){-M2_HUGE, -M2_HUGE};
    weld.angularHertz = M2_MAX_RATE;
    weld.linearHertz = M2_MAX_RATE;
    (void)Keep(&s, NewWeldJoint(s.world, &weld));
    ExpectFinite(&s, "long levers");
}

// Ratios at both ends against a body that cannot turn, a motor joint and
// a mouse at their largest budgets, a ratchet with the finest tooth.
static void TestRatiosAndDrives(void)
{
    Scene s = {ExtremeWorld(false), {{0}}, 0, {{0}}, 0, 0.0f};
    m2BodyId giant = Giant(&s, (m2Pos2){0.0, 0.0}, (m2Vec2){0.0f, 0.0f});
    m2BodyId speck = Speck(&s, (m2Pos2){2000.0, 0.0}, (m2Vec2){0.0f, 0.0f});
    m2Body_SetMotionLocks(s.world, giant, (m2MotionLocks){false, false, true});

    m2GearJointDef gear = m2DefaultGearJointDef();
    gear.bodyIdA = speck;
    gear.bodyIdB = giant;
    gear.ratio = M2_MIN_RATIO;
    (void)Keep(&s, NewGearJoint(s.world, &gear));
    gear.bodyIdA = giant;
    gear.bodyIdB = speck;
    gear.ratio = -M2_MAX_RATIO;
    (void)Keep(&s, NewGearJoint(s.world, &gear));

    m2PulleyJointDef pulley = m2DefaultPulleyJointDef();
    pulley.bodyIdA = giant;
    pulley.bodyIdB = speck;
    pulley.groundAnchorA = (m2Pos2){0.0, M2_JOINT_REACH};
    pulley.groundAnchorB = (m2Pos2){2000.0, -M2_JOINT_REACH};
    pulley.localAnchorA = (m2Vec2){M2_HUGE, 0.0f};
    pulley.ratio = M2_MIN_RATIO;
    (void)Keep(&s, NewPulleyJoint(s.world, &pulley));
    pulley.ratio = M2_MAX_RATIO;
    (void)Keep(&s, NewPulleyJoint(s.world, &pulley));

    m2MotorJointDef motor = m2DefaultMotorJointDef();
    motor.bodyIdA = giant;
    motor.bodyIdB = speck;
    motor.linearOffset = (m2Vec2){M2_HUGE, -M2_HUGE};
    motor.angularOffset = M2_MAX_ANGLE;
    motor.maxForce = M2_MAX_FORCE;
    motor.maxTorque = M2_MAX_FORCE;
    motor.correctionFactor = 1.0f;
    (void)Keep(&s, NewMotorJoint(s.world, &motor));

    m2MouseJointDef mouse = m2DefaultMouseJointDef();
    mouse.bodyIdA = giant;
    mouse.bodyIdB = speck;
    mouse.target = (m2Pos2){2000.0, 0.0};
    mouse.hertz = M2_MAX_RATE;
    mouse.dampingRatio = M2_MAX_COEFFICIENT;
    mouse.maxForce = M2_MAX_FORCE;
    m2JointId drag = Keep(&s, NewMouseJoint(s.world, &mouse));
    m2MouseJoint_SetTarget(s.world, drag, (m2Pos2){-M2_JOINT_REACH * 8.0, M2_JOINT_REACH * 8.0});

    m2RatchetJointDef ratchet = m2DefaultRatchetJointDef();
    ratchet.bodyIdA = speck;
    ratchet.bodyIdB = giant;
    ratchet.ratchet = M2_MIN_RATCHET_TOOTH;
    ratchet.phase = M2_MAX_ANGLE;
    (void)Keep(&s, NewRatchetJoint(s.world, &ratchet));
    ExpectFinite(&s, "ratios and drives");
}

// Water and wind at their strongest on a body whose inertia is as large
// as its mass is small, on a slab of the largest size that weighs and
// turns like a speck, half under the surface, and on a speck.
static void TestWaterAndWind(void)
{
    Scene s = {ExtremeWorld(false), {{0}}, 0, {{0}}, 0, M2_MAX_TIME_STEP};
    m2BodyId odd = Giant(&s, (m2Pos2){0.0, 0.0}, (m2Vec2){M2_MAX_SPEED, M2_MAX_SPEED});
    m2Body_SetMassData(s.world, odd, (m2MassData){M2_MIN_MASS, {0.0f, 0.0f}, M2_MAX_INERTIA});
    (void)Speck(&s, (m2Pos2){100.0, 0.0}, (m2Vec2){-M2_MAX_SPEED, M2_MAX_SPEED});
    m2BodyId slab = Add(&s, m2_dynamicBody, (m2Pos2){0.0, 2.0e4}, (m2Vec2){0.0f, 0.0f}, 0.0f);
    m2ShapeDef shape = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(M2_HUGE, M2_HUGE);
    (void)NewPolygonShape(s.world, slab, &shape, &box);
    m2Body_SetMassData(s.world, slab, (m2MassData){M2_MIN_MASS, {0.0f, 0.0f}, M2_MIN_INERTIA});
    m2FluidVolumeDef water = m2DefaultFluidVolumeDef();
    water.regionLower = (m2Pos2){-1.0e6, -1.0e6};
    water.regionUpper = (m2Pos2){1.0e6, 1.0e6};
    water.surface = 2.0e4; // through the slab's middle
    water.density = M2_MAX_DENSITY;
    water.linearDrag = M2_MAX_RATE;
    water.angularDrag = M2_MAX_RATE;
    water.flow = (m2Vec2){-M2_MAX_SPEED, M2_MAX_SPEED};
    (void)NewFluidVolume(s.world, &water);
    m2World_SetWind(s.world, (m2Vec2){M2_MAX_SPEED, -M2_MAX_SPEED}, M2_MAX_RATE);
    ExpectFinite(&s, "water and wind");
}

// The strongest particles at the largest speed against specks.
static void TestParticles(void)
{
    Scene s = {ExtremeWorld(true), {{0}}, 0, {{0}}, 0, 0.0f};
    (void)Speck(&s, (m2Pos2){0.0, 0.0}, (m2Vec2){0.0f, 0.0f});
    (void)Giant(&s, (m2Pos2){0.0, -501.0}, (m2Vec2){0.0f, M2_MAX_SPEED});
    m2Polygon blob = m2MakeBox(0.2f, 0.2f);
    uint32_t flags =
        m2_springParticle | m2_elasticParticle | m2_viscousParticle | m2_tensileParticle;
    (void)m2World_FillPolygonWithParticles(s.world, &blob, (m2Pos2){0.0, 0.3},
                                           (m2Vec2){M2_MAX_SPEED, -M2_MAX_SPEED}, flags);
    ExpectFinite(&s, "particles");
}

#define SWEEP 16

// Joints that saturate at M2_IMPULSE_LIMIT in every direction: a mouse
// (a point pair scaled to its limit) and a slider (a row pair clamped
// row by row) on giants under the strongest gravity, one per direction
// of a sweep around the circle. The stored impulse is formed by scaling
// or by adding a clamped difference, either of which can round a float
// past the limit; the world must restore its own snapshot after every
// step.
static void TestSaturatedPairs(void)
{
    m2World* world = ExtremeWorld(false);
    m2BodyDef groundDef = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &groundDef);
    Scene s = {world, {{0}}, 0, {{0}}, 0, 0.0f};
    for (int32_t i = 0; i < SWEEP; ++i)
    {
        float angle = 6.2831853f * (float)i / (float)SWEEP + 0.1f;
        m2Vec2 direction = {cosf(angle), sinf(angle)};
        s.bodyCount = 0;
        m2BodyId pulled = Giant(&s, (m2Pos2){5000.0 * (double)i, 0.0}, (m2Vec2){0.0f, 0.0f});
        m2MouseJointDef mouse = m2DefaultMouseJointDef();
        mouse.bodyIdA = ground;
        mouse.bodyIdB = pulled;
        mouse.target =
            (m2Pos2){5000.0 * (double)i + 1.0e6 * (double)direction.x, 1.0e6 * (double)direction.y};
        mouse.hertz = M2_MAX_RATE;
        mouse.dampingRatio = M2_MAX_COEFFICIENT;
        mouse.maxForce = M2_MAX_FORCE;
        (void)NewMouseJoint(world, &mouse);
        m2BodyId slid = Giant(&s, (m2Pos2){5000.0 * (double)i, 1.0e5}, (m2Vec2){0.0f, 0.0f});
        m2PrismaticJointDef slider = m2DefaultPrismaticJointDef();
        slider.bodyIdA = ground;
        slider.bodyIdB = slid;
        slider.localAnchorA = (m2Vec2){0.0f, 0.0f};
        slider.localAxisA = direction;
        (void)NewPrismaticJoint(world, &slider);
    }
    bool valid = true;
    for (int32_t step = 0; step < 30 && valid; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        valid = m2World_Validate(world) && RoundTrips(world);
    }
    CHECK(valid, "joints saturated in every direction restore their own snapshot every step");
    m2DestroyWorld(world);
}

// Particles faster than the speed cap in every direction, at the shortest
// step, where the cap is M2_MAX_SPEED: each velocity is scaled to the cap
// and must stay within it, so the world restores its own snapshot.
static void TestParticlesAtTheCap(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.particleCapacity = 4 * SWEEP;
    def.particleRadius = 10.0f; // a diameter per shortest step is 2e7 m/s
    m2World* world = NewWorld(&def);
    for (int32_t i = 0; i < 4 * SWEEP; ++i)
    {
        float angle = 6.2831853f * (float)i / (float)(4 * SWEEP);
        float c = cosf(angle);
        float sn = sinf(angle);
        float longest = fabsf(c) > fabsf(sn) ? fabsf(c) : fabsf(sn);
        m2Vec2 velocity = {M2_MAX_SPEED * c / longest, M2_MAX_SPEED * sn / longest};
        EmitParticle(world, (m2Pos2){100.0 * (double)i, 0.0}, velocity, 0);
    }
    bool valid = true;
    for (int32_t step = 0; step < 4 && valid; ++step)
    {
        m2World_Step(world, M2_MIN_TIME_STEP, 1);
        valid = m2World_Validate(world) && RoundTrips(world);
    }
    CHECK(valid, "particles at the speed cap restore their own snapshot every step");
    m2DestroyWorld(world);
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
#if defined(__GLIBC__) && !defined(NDEBUG)
    (void)feenableexcept(FE_OVERFLOW | FE_INVALID | FE_DIVBYZERO);
#endif
    TestGiantOnSpecks();
    TestGiantResting();
    TestLongLevers();
    TestHangingGiant();
    TestRatiosAndDrives();
    TestWaterAndWind();
    TestParticles();
    TestSaturatedPairs();
    TestParticlesAtTheCap();
    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
