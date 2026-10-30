// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Query gate: the closest ray cast picks the right shape with an
// analytically exact fraction, works on every geometry, and stays
// exact far from the origin; AABB overlap returns exactly the
// expected shapes in canonical order with a truthful total; queries
// never move the world hash; and a query storm's results hash
// identically across CI cells.

#include "helpers.h"
#include "test_harness.h"

#include "maul2d/base.h"
#include "maul2d/maul2d.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m2ShapeId AddStaticBox(m2World* world, double x, double y, float hw, float hh)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.position = (m2Pos2){x, y};
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(hw, hh);
    return NewPolygonShape(world, body, &sd, &box);
}

static m2ShapeId AddStaticCircle(m2World* world, double x, double y, float radius)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.position = (m2Pos2){x, y};
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle c = {{0.0f, 0.0f}, radius};
    return NewCircleShape(world, body, &sd, &c);
}

static bool SameShape(m2ShapeId a, m2ShapeId b)
{
    return a.index1 == b.index1 && a.generation == b.generation;
}

// The query pack: convex sweeps and overlaps through one
// GJK kernel. Analytic fractions, the ray conventions (closest wins,
// ties to lower index, fraction 0 with zero normal on initial
// overlap), and the one-sided chain law extended to sweeps.
static void TestShapeCastsAndOverlaps(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -0.5};
    m2BodyId ground = NewBody(world, &gd);
    m2ShapeDef gs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(10.0f, 0.5f); // top face at y = 0
    m2ShapeId floor = NewPolygonShape(world, ground, &gs, &slab);

    m2Transform pose = {{0.0, 5.0}, {1.0f, 0.0f}};
    m2QueryFilter all = m2DefaultQueryFilter();

    // Circle drop: contact when the center reaches y = radius, so the
    // fraction is about (5 - 0.5) / 10 (the slop skin shifts it a hair).
    m2Circle ball = {{0.0f, 0.0f}, 0.5f};
    CastResult drop = CircleClosest(world, &ball, pose, (m2Vec2){0.0f, -10.0f}, all);
    CHECK(drop.hit, "the falling circle finds the floor");
    CHECK(drop.fraction > 0.43f && drop.fraction < 0.46f, "at the analytic fraction");
    CHECK(drop.normal.y > 0.99f, "with the surface normal facing the sweep");
    CHECK(drop.shapeId.index1 == floor.index1, "and names the floor");

    // Capsule and polygon sweeps land on the same face.
    m2Capsule pill = {{-0.4f, 0.0f}, {0.4f, 0.0f}, 0.3f};
    CastResult pillDrop = CapsuleClosest(world, &pill, pose, (m2Vec2){0.0f, -10.0f}, all);
    CHECK(pillDrop.hit && pillDrop.fraction > 0.44f && pillDrop.fraction < 0.48f,
          "the capsule lands where its radius says");
    m2Polygon crate = m2MakeBox(0.4f, 0.4f);
    CastResult crateDrop = PolygonClosest(world, &crate, pose, (m2Vec2){0.0f, -10.0f}, all);
    CHECK(crateDrop.hit && crateDrop.fraction > 0.44f && crateDrop.fraction < 0.47f,
          "the box lands on its face");

    // Initial overlap: pose the circle inside the floor.
    m2Transform buried = {{0.0, -0.5}, {1.0f, 0.0f}};
    CastResult inside = CircleClosest(world, &ball, buried, (m2Vec2){0.0f, -1.0f}, all);
    CHECK(inside.hit && inside.fraction == 0.0f && inside.normal.x == 0.0f &&
              inside.normal.y == 0.0f,
          "initial overlap follows the ray convention");

    // A miss stays a miss.
    CastResult away = CircleClosest(world, &ball, pose, (m2Vec2){0.0f, 8.0f}, all);
    CHECK(!away.hit, "sweeping away hits nothing");

    // One-sided chain: solid side up. From below, the sweep ghosts
    // through; from above, it lands.
    m2BodyDef cd = m2DefaultBodyDef();
    cd.position = (m2Pos2){30.0, 0.0};
    m2BodyId shelfBody = NewBody(world, &cd);
    m2Vec2 pts[5] = {{4.0f, 2.0f}, {2.0f, 2.0f}, {0.0f, 2.0f}, {-2.0f, 2.0f}, {-4.0f, 2.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 5;
    NewChain(world, shelfBody, &chain);
    m2Transform below = {{30.0, -2.0}, {1.0f, 0.0f}};
    CastResult rising = CircleClosest(world, &ball, below, (m2Vec2){0.0f, 8.0f}, all);
    CHECK(!rising.hit, "sweeps from the ghost side pass through the shelf");
    m2Transform above = {{30.0, 6.0}, {1.0f, 0.0f}};
    CastResult diving = CircleClosest(world, &ball, above, (m2Vec2){0.0f, -8.0f}, all);
    CHECK(diving.hit, "and land from the solid side");

    // Overlaps: a probe circle around the origin touches the floor,
    // truthfully counts, and reports ascending.
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.45};
    m2BodyId box = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2ShapeId boxShape = NewPolygonShape(world, box, &sd, &unit);
    m2Transform probe = {{0.0, 0.2}, {1.0f, 0.0f}};
    m2Circle area = {{0.0f, 0.0f}, 1.0f};
    CHECK(m2World_OverlapCircle(world, &area, probe, all, NULL, 0) == 2,
          "the probe touches floor and box, truthfully counted");
    m2ShapeId found[4];
    int32_t n = m2World_OverlapCircle(world, &area, probe, all, found, 4);
    CHECK(n == 2 && found[0].index1 < found[1].index1, "ascending slot order");
    CHECK(found[0].index1 == floor.index1 && found[1].index1 == boxShape.index1,
          "and the right two");

    // A far probe touches nothing; a capsule probe agrees with the
    // circle; the polygon probe sees only the box when small.
    m2Transform distant = {{50.0, 50.0}, {1.0f, 0.0f}};
    CHECK(m2World_OverlapCircle(world, &area, distant, all, NULL, 0) == 0, "far probe is empty");
    m2Transform tight = {{0.0, 0.45}, {1.0f, 0.0f}};
    m2Polygon smallProbe = m2MakeBox(0.1f, 0.1f);
    CHECK(m2World_OverlapPolygon(world, &smallProbe, tight, all, NULL, 0) == 1,
          "a tight polygon probe sees only the box");

    m2DestroyWorld(world);
}

// All-hits variants (parity bucket 2): every shape along the path,
// ascending fractions, truthful totals, closest kept under pressure.
static void TestAllHits(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    m2World* world = NewWorld(&def);
    m2QueryFilter all = m2DefaultQueryFilter();

    // Three walls in a row.
    m2ShapeId walls[3];
    for (int32_t i = 0; i < 3; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.position = (m2Pos2){2.0 + (double)i * 3.0, 0.0};
        m2BodyId post = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon slab = m2MakeBox(0.3f, 2.0f);
        walls[i] = NewPolygonShape(world, post, &sd, &slab);
    }
    m2CastHit hits[8];
    int32_t n = m2World_CastRayAll(world, (m2Pos2){-2.0, 0.0}, (m2Vec2){12.0f, 0.0f}, all, hits, 8);
    CHECK(n == 3, "the ray reports all three walls");
    CHECK(hits[0].fraction < hits[1].fraction && hits[1].fraction < hits[2].fraction,
          "in ascending fraction order");
    CHECK(hits[0].shapeId.index1 == walls[0].index1 && hits[2].shapeId.index1 == walls[2].index1,
          "closest first, farthest last");

    // Under capacity pressure the total stays truthful and the two
    // CLOSEST are the ones kept.
    m2CastHit two[2];
    int32_t pressed =
        m2World_CastRayAll(world, (m2Pos2){-2.0, 0.0}, (m2Vec2){12.0f, 0.0f}, all, two, 2);
    CHECK(pressed == 3, "truthful total under pressure");
    CHECK(two[0].shapeId.index1 == walls[0].index1 && two[1].shapeId.index1 == walls[1].index1,
          "and the closest two are kept");
    CHECK(m2World_CastRayAll(world, (m2Pos2){-2.0, 0.0}, (m2Vec2){12.0f, 0.0f}, all, NULL, 0) == 3,
          "counting works without an array");

    // A swept circle collects the same walls.
    m2Circle ball = {{0.0f, 0.0f}, 0.4f};
    m2Transform pose = {{-2.0, 0.0}, {1.0f, 0.0f}};
    n = m2World_CastCircleAll(world, &ball, pose, (m2Vec2){12.0f, 0.0f}, all, hits, 8);
    CHECK(n == 3, "the swept circle reports all three too");
    CHECK(hits[0].fraction < hits[1].fraction, "sorted the same way");

    // One-sided chain: rising sweeps never count it, diving ones do.
    m2BodyDef cd = m2DefaultBodyDef();
    cd.position = (m2Pos2){20.0, 0.0};
    m2BodyId shelfBody = NewBody(world, &cd);
    m2Vec2 pts[5] = {{2.5f, 2.0f}, {1.2f, 2.0f}, {0.0f, 2.0f}, {-1.2f, 2.0f}, {-2.5f, 2.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 5;
    NewChain(world, shelfBody, &chain);
    m2Transform below = {{20.0, -1.0}, {1.0f, 0.0f}};
    CHECK(m2World_CastCircleAll(world, &ball, below, (m2Vec2){0.0f, 6.0f}, all, NULL, 0) == 0,
          "rising sweeps ghost through the shelf");
    m2Transform above = {{20.0, 5.0}, {1.0f, 0.0f}};
    // The dive lands on the shared vertex of the two live links,
    // so both count: all-hits means ALL.
    CHECK(m2World_CastCircleAll(world, &ball, above, (m2Vec2){0.0f, -6.0f}, all, NULL, 0) == 2,
          "diving sweeps count both touched links");

    m2DestroyWorld(world);
}

// The mover kit: planes in a corner, the solver respecting both,
// velocity clipped against what was hit, and the chain law held.
static void TestMoverKit(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    m2World* world = NewWorld(&def);
    m2QueryFilter all = m2DefaultQueryFilter();

    // A floor and a wall meeting at the origin corner.
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &gd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(8.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);
    m2BodyDef wd = m2DefaultBodyDef();
    wd.position = (m2Pos2){4.5, 2.0};
    m2BodyId wall = NewBody(world, &wd);
    m2Polygon post = m2MakeBox(0.5f, 2.5f);
    NewPolygonShape(world, wall, &fs, &post);

    // A capsule mover standing on the floor near the wall.
    m2Capsule mover = {{0.0f, -0.4f}, {0.0f, 0.4f}, 0.3f};
    m2Transform pose = {{3.69, 0.71}, {1.0f, 0.0f}};
    m2MoverPlane found[8];
    int32_t n = m2World_CollideMover(world, &mover, pose, all, found, 8);
    CHECK(n == 2, "the corner offers two planes");
    CHECK(found[0].shapeId.index1 < found[1].shapeId.index1, "ascending shape order");
    // One normal points up (the floor), one points -x (the wall).
    bool sawFloor = false;
    bool sawWall = false;
    for (int32_t i = 0; i < n; ++i)
    {
        sawFloor = sawFloor || found[i].normal.y > 0.9f;
        sawWall = sawWall || found[i].normal.x < -0.9f;
    }
    CHECK(sawFloor && sawWall, "with the right orientations");

    // Drive into the corner: the solver returns the translation closest
    // to the wish that clears both planes.
    m2Vec2 wish = {0.5f, -0.3f}; // into the wall AND the floor
    m2MoverMove solved = m2SolveMover(wish, found, n);
    float floorLeak = 0.0f;
    float wallLeak = 0.0f;
    for (int32_t i = 0; i < n; ++i)
    {
        float after = found[i].separation + solved.translation.x * found[i].normal.x +
                      solved.translation.y * found[i].normal.y;
        if (found[i].normal.y > 0.9f)
        {
            floorLeak = after;
        }
        else
        {
            wallLeak = after;
        }
    }
    CHECK(floorLeak > -1.0e-4f && wallLeak > -1.0e-4f,
          "the solved translation stays out of both planes");
    CHECK(solved.pressed != 0, "the move rests on what stopped it");

    // A clear wish passes untouched.
    m2MoverMove away = m2SolveMover((m2Vec2){-0.5f, 0.3f}, found, n);
    CHECK(away.translation.x == -0.5f && away.translation.y == 0.3f && away.pressed == 0,
          "a move away from every plane is not touched");

    // Velocity clipping: the into-corner velocity loses its normal
    // components, keeps what slides.
    m2Vec2 clipped = m2ClipMoverVelocity((m2Vec2){2.0f, -1.0f}, found, n, solved.pressed);
    CHECK(clipped.x < 0.1f && clipped.x >= -1.0e-4f, "the wall eats the x approach");
    CHECK(clipped.y >= -1.0e-4f, "the floor eats the downward part");

    // Free air: no planes at all.
    m2Transform sky = {{-5.0, 5.0}, {1.0f, 0.0f}};
    CHECK(m2World_CollideMover(world, &mover, sky, all, NULL, 0) == 0, "clear air is clear");

    // One-way shelf: standing under it collects nothing, standing on
    // it collects the shelf.
    m2BodyDef cd = m2DefaultBodyDef();
    cd.position = (m2Pos2){-4.0, 2.0};
    m2BodyId shelfBody = NewBody(world, &cd);
    m2Vec2 pts[5] = {{2.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 0.0f}, {-1.0f, 0.0f}, {-2.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 5;
    NewChain(world, shelfBody, &chain);
    m2Transform under = {{-4.5, 1.3}, {1.0f, 0.0f}};
    CHECK(m2World_CollideMover(world, &mover, under, all, NULL, 0) == 0,
          "under the shelf the mover is a ghost");
    m2Transform standing = {{-4.5, 2.71}, {1.0f, 0.0f}}; // mid-link, one plane
    CHECK(m2World_CollideMover(world, &mover, standing, all, NULL, 0) == 1,
          "standing on it collects the shelf plane");

    m2DestroyWorld(world);
}

static void TestRayClosest(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);

    // Three walls along +x; the ray must pick the nearest.
    AddStaticBox(world, 8.0, 0.0, 0.5, 2.0);
    m2ShapeId nearWall = AddStaticBox(world, 4.0, 0.0, 0.5, 2.0);
    AddStaticBox(world, 12.0, 0.0, 0.5, 2.0);

    CastResult hit =
        RayClosest(world, (m2Pos2){0.0, 0.0}, (m2Vec2){20.0f, 0.0f}, m2DefaultQueryFilter());
    CHECK(hit.hit, "ray hits");
    CHECK(SameShape(hit.shapeId, nearWall), "closest wall wins");
    // Wall face at x = 3.5, ray length 20: fraction = 0.175 exactly.
    CHECK(hit.fraction == 0.175f, "box fraction is analytically exact");
    CHECK(hit.normal.x == -1.0f && hit.normal.y == 0.0f, "face normal points back");
    double px = hit.point.x - 3.5;
    CHECK(px > -1.0e-6 && px < 1.0e-6 && hit.point.y == 0.0, "hit point lands on the face");

    // A miss stays a miss.
    CastResult miss =
        RayClosest(world, (m2Pos2){0.0, 10.0}, (m2Vec2){20.0f, 0.0f}, m2DefaultQueryFilter());
    CHECK(!miss.hit, "ray over the walls misses");

    // Circle fraction: center (4,-6) r=1, vertical ray from (4,-3)
    // (below the wall) down 10: hits at y=-5, fraction 0.2.
    AddStaticCircle(world, 4.0, -6.0, 1.0f);
    CastResult round =
        RayClosest(world, (m2Pos2){4.0, -3.0}, (m2Vec2){0.0f, -10.0f}, m2DefaultQueryFilter());
    CHECK(round.hit, "circle hit");
    CHECK(round.fraction == 0.2f, "circle fraction is analytically exact");
    CHECK(round.normal.y == 1.0f, "circle normal points up");

    m2DestroyWorld(world);
}

static void TestRayGeometries(void)
{
    // Capsule and segment kernels answer too.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.position = (m2Pos2){0.0, 0.0};
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Capsule capsule = {{-1.0f, 0.0f}, {1.0f, 0.0f}, 0.5f};
    NewCapsuleShape(world, body, &sd, &capsule);

    CastResult hit =
        RayClosest(world, (m2Pos2){0.0, 5.0}, (m2Vec2){0.0f, -10.0f}, m2DefaultQueryFilter());
    CHECK(hit.hit, "capsule hit");
    CHECK(hit.fraction == 0.45f, "capsule top at y=0.5: fraction exact");

    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){10.0, 0.0};
    m2BodyId ground = NewBody(world, &gd);
    m2Segment segment = {{-2.0f, 0.0f}, {2.0f, 0.0f}};
    NewSegmentShape(world, ground, &sd, &segment);

    CastResult seg =
        RayClosest(world, (m2Pos2){10.0, 3.0}, (m2Vec2){0.0f, -6.0f}, m2DefaultQueryFilter());
    CHECK(seg.hit, "segment hit");
    CHECK(seg.fraction == 0.5f, "segment fraction exact");

    m2DestroyWorld(world);
}

static void TestRayFarFromOrigin(void)
{
    // The f64 discipline: the same cast, 3e5 meters out, same numbers.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    double bx = 3.0e5;
    AddStaticBox(world, bx + 4.0, 0.0, 0.5, 2.0);
    CastResult hit =
        RayClosest(world, (m2Pos2){bx, 0.0}, (m2Vec2){20.0f, 0.0f}, m2DefaultQueryFilter());
    CHECK(hit.hit, "far cast hits");
    CHECK(hit.fraction == 0.175f, "far fraction identical to the origin scene");
    double farPx = hit.point.x - (bx + 3.5);
    CHECK(farPx > -1.0e-5 && farPx < 1.0e-5, "far hit point exact in f64");

    m2DestroyWorld(world);
}

static void TestFarAndLongRaysAtRoundShapes(void)
{
    // A ray from far away, or a very long one, at a circle and a capsule
    // hits the face it crosses: the normal points back along the ray and
    // the fraction is where that face lies, never 0 at the ray's origin.
    const double backs[] = {1.0e9, 1.0e10, 1.0e15, 1.0e18, 10.0};
    const float lengths[] = {2.0e9f, 2.0e10f, 2.0e15f, 2.0e18f, 1.0e20f};
    for (int32_t kind = 0; kind < 2; ++kind)
    {
        m2WorldDef def = m2DefaultWorldDef();
        m2World* world = NewWorld(&def);
        m2BodyDef bd = m2DefaultBodyDef();
        m2BodyId body = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Circle circle = {{0.0f, 0.0f}, 1.0f};
        m2Capsule capsule = {{0.0f, -1.0f}, {0.0f, 1.0f}, 0.2f};
        double face = kind == 0 ? 0.99498743710662 : 0.2; // where y = 0.1 crosses the shape
        if (kind == 0)
        {
            NewCircleShape(world, body, &sd, &circle);
        }
        else
        {
            NewCapsuleShape(world, body, &sd, &capsule);
        }
        for (int32_t k = 0; k < 5; ++k)
        {
            CastResult hit = RayClosest(world, (m2Pos2){-backs[k], 0.1}, (m2Vec2){lengths[k], 0.0f},
                                        m2DefaultQueryFilter());
            double expected = (backs[k] - face) / (double)lengths[k];
            double error = (double)hit.fraction - expected;
            CHECK(hit.hit && hit.normal.x < -0.9f && hit.fraction > 0.0f &&
                      error < 1.0e-6 * expected && error > -1.0e-6 * expected,
                  kind == 0 ? "a far or long ray hits the circle's near face"
                            : "a far or long ray hits the capsule's near face");
        }
        m2DestroyWorld(world);
    }
}

static void TestRayFromTheSurfaceHitsAtZero(void)
{
    // An origin one float step outside a unit circle, aimed in: the root
    // rounds a few ulps below 0 unless it is held there.
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle circle = {{0.0f, 0.0f}, 1.0f};
    NewCircleShape(world, body, &sd, &circle);
    m2Pos2 origin = {(double)-0x1.b9c784p-1f, (double)0x1.02cb9cp-1f};
    m2Vec2 translation = {0x1.664cdap+0f, 0x1.a4ae84p-2f};
    CastResult hit = RayClosest(world, origin, translation, m2DefaultQueryFilter());
    CHECK(hit.hit && hit.fraction == 0.0f, "a ray from the surface hits at a fraction of 0");
    m2DestroyWorld(world);
}

static void TestOverlap(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);

    m2ShapeId a = AddStaticBox(world, 0.0, 0.0, 0.5, 0.5);
    m2ShapeId b = AddStaticBox(world, 2.0, 0.0, 0.5, 0.5);
    m2ShapeId c = AddStaticCircle(world, 4.0, 0.0, 0.5f);
    AddStaticBox(world, 40.0, 0.0, 0.5, 0.5); // far away

    m2ShapeId results[8];
    int32_t total = m2World_OverlapAabb(world, (m2WorldAabb){{-1.0, -1.0}, {4.2, 1.0}},
                                        m2DefaultQueryFilter(), results, 8);
    CHECK(total == 3, "overlap finds exactly the three");
    CHECK(SameShape(results[0], a) && SameShape(results[1], b) && SameShape(results[2], c),
          "overlap order is canonical (creation order)");

    // Truthful total under a tight capacity; the kept ones are the
    // lowest indices.
    m2ShapeId two[2];
    int32_t clamped = m2World_OverlapAabb(world, (m2WorldAabb){{-1.0, -1.0}, {4.2, 1.0}},
                                          m2DefaultQueryFilter(), two, 2);
    CHECK(clamped == 3, "total reported even beyond capacity");
    CHECK(SameShape(two[0], a) && SameShape(two[1], b), "capacity keeps the canonical head");

    // Empty region.
    int32_t none = m2World_OverlapAabb(world, (m2WorldAabb){{100.0, 100.0}, {101.0, 101.0}},
                                       m2DefaultQueryFilter(), results, 8);
    CHECK(none == 0, "empty region reports zero");

    m2DestroyWorld(world);
}

static void TestQueriesAreReadOnly(void)
{
    // A query storm between steps must not move the simulation hash.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m2World* world = NewWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);
    for (int32_t i = 0; i < 8; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-3.0 + 0.9 * (double)i, 1.0 + 0.5 * (double)(i % 3)};
        m2BodyId body = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Circle circle = {{0.0f, 0.0f}, 0.3f};
        NewCircleShape(world, body, &sd, &circle);
    }
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    uint64_t before = m2World_Hash(world);
    m2ShapeId results[32];
    for (int32_t i = 0; i < 50; ++i)
    {
        RayClosest(world, (m2Pos2){-5.0 + 0.2 * (double)i, 3.0}, (m2Vec2){0.3f, -6.0f},
                   m2DefaultQueryFilter());
        m2World_OverlapAabb(world, (m2WorldAabb){{-4.0, -1.0}, {0.1 * (double)i, 2.0}},
                            m2DefaultQueryFilter(), results, 32);
    }
    CHECK(m2World_Hash(world) == before, "queries never touch simulation state");

    uint64_t hashesA[20];
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashesA[i] = m2World_Hash(world);
    }
    m2DestroyWorld(world);

    // Same scene, no query storm: identical trajectory.
    world = NewWorld(&def);
    floor = NewBody(world, &fd);
    NewPolygonShape(world, floor, &fs, &slab);
    for (int32_t i = 0; i < 8; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-3.0 + 0.9 * (double)i, 1.0 + 0.5 * (double)(i % 3)};
        m2BodyId body = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Circle circle = {{0.0f, 0.0f}, 0.3f};
        NewCircleShape(world, body, &sd, &circle);
    }
    for (int32_t i = 0; i < 50; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t tail = m2World_Hash(world);
    CHECK(tail == hashesA[19], "trajectory with and without queries is identical");
    m2DestroyWorld(world);
}

static void TestQueryFilters(void)
{
    // A filtered ray sees only what its mask allows; the overlap list
    // shrinks the same way; the default filter behaves as before.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.position = (m2Pos2){4.0, 0.0};
    m2BodyId wallBody = NewBody(world, &bd);
    m2ShapeDef wallShape = m2DefaultShapeDef();
    wallShape.categoryBits = 0x2; // "wall"
    m2Polygon wall = m2MakeBox(0.5, 2.0f);
    m2ShapeId wallId = NewPolygonShape(world, wallBody, &wallShape, &wall);

    bd.position = (m2Pos2){2.0, 0.0};
    m2BodyId ghostBody = NewBody(world, &bd);
    m2ShapeDef ghostShape = m2DefaultShapeDef();
    ghostShape.categoryBits = 0x4; // "ghost"
    m2Circle ghost = {{0.0f, 0.0f}, 0.4f};
    NewCircleShape(world, ghostBody, &ghostShape, &ghost);

    // Walls-only ray passes through the ghost and hits the wall.
    m2QueryFilter wallsOnly = {1u, 0x2u};
    CastResult hit = RayClosest(world, (m2Pos2){0.0, 0.0}, (m2Vec2){10.0f, 0.0f}, wallsOnly);
    CHECK(hit.hit && hit.shapeId.index1 == wallId.index1, "filtered ray skips the ghost");

    // The default filter hits the ghost first.
    CastResult any =
        RayClosest(world, (m2Pos2){0.0, 0.0}, (m2Vec2){10.0f, 0.0f}, m2DefaultQueryFilter());
    CHECK(any.hit && any.shapeId.index1 != wallId.index1, "default filter sees everything");

    m2ShapeId results[4];
    int32_t total =
        m2World_OverlapAabb(world, (m2WorldAabb){{0.0, -1.0}, {5.0, 1.0}}, wallsOnly, results, 4);
    CHECK(total == 1 && results[0].index1 == wallId.index1, "filtered overlap lists walls only");

    m2DestroyWorld(world);
}

static void TestContactData(void)
{
    // A box resting on a slab reports one touching contact with an
    // upward normal and load-bearing impulses.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(5.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.45};
    m2BodyId box = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, box, &sd, &unit);

    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    m2ContactData data[4];
    int32_t total = m2World_GetContactData(world, data, 4);
    CHECK(total == 1, "one touching contact");
    CHECK(data[0].pointCount == 2, "box on slab carries two points");
    float ny = data[0].normal.y;
    CHECK(ny == 1.0f || ny == -1.0f, "contact normal is vertical");
    CHECK(data[0].normalImpulses[0] > 0.0f && data[0].normalImpulses[1] > 0.0f,
          "the contact carries the box's weight");

    // Capacity clamp stays truthful.
    CHECK(m2World_GetContactData(world, data, 0) == 1, "total reported beyond capacity");
    CHECK(m2World_GetContactData(world, NULL, 4) == 1,
          "a null array measures, whatever the capacity");

    m2DestroyWorld(world);
}

static void TestAllHitsBeyondSixtyFour(void)
{
    // A ray through 100 thin posts: every hit fits a 128-entry array,
    // so all 100 come back in order, none dropped.
    enum
    {
        POSTS = 100
    };
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = POSTS + 4;
    def.shapeCapacity = POSTS + 4;
    m2World* world = NewWorld(&def);
    for (int32_t i = 0; i < POSTS; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.position = (m2Pos2){1.0 + (double)i, 0.0};
        m2BodyId post = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon slab = m2MakeBox(0.1f, 1.0f);
        NewPolygonShape(world, post, &sd, &slab);
    }
    static m2CastHit hits[128];
    m2Circle ball = {{0.0f, 0.0f}, 0.2f};
    m2Transform pose = {{0.0, 0.0}, {1.0f, 0.0f}};
    int32_t rays =
        m2World_CastRayAll(world, (m2Pos2){0.0, 0.0}, (m2Vec2){(float)POSTS + 1.0f, 0.0f},
                           m2DefaultQueryFilter(), hits, 128);
    bool ordered = true;
    for (int32_t i = 1; i < rays && i < 128; ++i)
    {
        ordered = ordered && hits[i - 1].fraction < hits[i].fraction;
    }
    CHECK(rays == POSTS && ordered, "a ray keeps all 100 hits in order");
    int32_t circles = m2World_CastCircleAll(world, &ball, pose, (m2Vec2){(float)POSTS + 1.0f, 0.0f},
                                            m2DefaultQueryFilter(), hits, 128);
    CHECK(circles == POSTS && hits[99].shapeId.index1 != 0, "a swept circle keeps all 100 too");
    m2DestroyWorld(world);
}

// A ray that starts inside a rounded polygon hits where it starts, with
// no normal, like every other solid shape; from outside it meets the
// rounded corner.
static void TestRayInsideRoundedPolygon(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    m2BodyId body = NewBody(world, &bd);
    m2Vec2 square[4] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}};
    m2Polygon rounded = MakePolygon(square, 4, 0.5f);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2ShapeId shape = NewPolygonShape(world, body, &sd, &rounded);
    CastResult inside = ShapeRay(world, shape, (m2Pos2){0.0, 0.0}, (m2Vec2){5.0f, 0.0f});
    CHECK(inside.hit && inside.fraction == 0.0f, "a ray from inside hits at its start");
    CHECK(inside.normal.x == 0.0f && inside.normal.y == 0.0f, "with no normal");
    CastResult corner = ShapeRay(world, shape, (m2Pos2){3.0, 3.0}, (m2Vec2){-3.0f, -3.0f});
    double expected = 1.0 + 0.5 * 0.70710678;
    CHECK(corner.hit && fabs(corner.point.x - expected) < 1.0e-4 &&
              fabs(corner.point.y - expected) < 1.0e-4,
          "a diagonal ray meets the rounded corner");
    CHECK(fabsf(corner.normal.x - 0.70710678f) < 1.0e-4f, "on the corner's normal");
    m2DestroyWorld(world);
}

enum
{
    CLOSEST_CASTS = 5
};

// Runs every closest cast from origin along translation, each into its own
// hit filled first with a pattern no cast writes, and counts the hits.
static int32_t CastEveryClosest(const m2World* world, m2ShapeId shape, m2Pos2 origin,
                                m2Vec2 translation, m2CastHit* hits)
{
    m2QueryFilter all = m2DefaultQueryFilter();
    m2Circle ball = {{0.0f, 0.0f}, 0.25f};
    m2Capsule rod = {{-0.25f, 0.0f}, {0.25f, 0.0f}, 0.1f};
    m2Polygon chip = m2MakeBox(0.25f, 0.25f);
    m2Transform pose = {origin, {1.0f, 0.0f}};
    memset(hits, 0x5A, sizeof(m2CastHit) * CLOSEST_CASTS);
    int32_t count = 0;
    count += m2World_CastRayClosest(world, origin, translation, all, &hits[0]) ? 1 : 0;
    count += m2World_CastCircleClosest(world, &ball, pose, translation, all, &hits[1]) ? 1 : 0;
    count += m2World_CastCapsuleClosest(world, &rod, pose, translation, all, &hits[2]) ? 1 : 0;
    count += m2World_CastPolygonClosest(world, &chip, pose, translation, all, &hits[3]) ? 1 : 0;
    count += m2Shape_CastRay(world, shape, origin, translation, &hits[4]) ? 1 : 0;
    return count;
}

// Whether every field of the count hits is zero, with the null shape id.
static bool HitsZero(const m2CastHit* hits, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        const m2CastHit* h = &hits[i];
        if (h->shapeId.index1 != 0 || h->shapeId.generation != 0 || h->point.x != 0.0 ||
            h->point.y != 0.0 || h->normal.x != 0.0f || h->normal.y != 0.0f || h->fraction != 0.0f)
        {
            return false;
        }
    }
    return true;
}

// A closest cast says whether it hits and writes the hit. A miss, a
// refusal and a null world leave zeros, so a caller never reads an earlier
// hit as the answer, and a null hitOut is refused as a misuse.
static void TestCastHitOut(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(2.0f, 0.5f);
    m2ShapeId floor = NewPolygonShape(world, ground, &sd, &slab);
    m2Pos2 above = {0.0, 3.0};
    m2Vec2 down = {0.0f, -6.0f};
    m2CastHit hits[CLOSEST_CASTS];

    CHECK(CastEveryClosest(world, floor, above, down, hits) == CLOSEST_CASTS,
          "every cast that hits says so");
    bool wrote = true;
    for (int32_t i = 0; i < CLOSEST_CASTS; ++i)
    {
        wrote = wrote && hits[i].shapeId.index1 == floor.index1 && hits[i].normal.y == 1.0f;
    }
    CHECK(wrote, "and writes the hit");

    uint64_t misuse = m2World_GetCounters(world).misuseCount;
    CHECK(CastEveryClosest(world, floor, (m2Pos2){9.0, 3.0}, down, hits) == 0 &&
              HitsZero(hits, CLOSEST_CASTS),
          "a miss leaves zeros");
    CHECK(m2World_GetCounters(world).misuseCount == misuse, "and counts no misuse");

    CHECK(CastEveryClosest(world, floor, (m2Pos2){(double)NAN, 3.0}, down, hits) == 0 &&
              HitsZero(hits, CLOSEST_CASTS),
          "a refused cast leaves zeros");
    CHECK(m2World_GetCounters(world).misuseCount == misuse + CLOSEST_CASTS,
          "each counting one misuse");

    m2ShapeId stale = {floor.index1, floor.generation + 1u};
    m2CastHit staleHit;
    memset(&staleHit, 0x5A, sizeof(staleHit));
    CHECK(!m2Shape_CastRay(world, stale, above, down, &staleHit) && HitsZero(&staleHit, 1),
          "a cast at a stale shape leaves zeros");

    CHECK(CastEveryClosest(NULL, floor, above, down, hits) == 0 && HitsZero(hits, CLOSEST_CASTS),
          "a null world leaves zeros");

    m2QueryFilter all = m2DefaultQueryFilter();
    m2Circle ball = {{0.0f, 0.0f}, 0.25f};
    m2Capsule rod = {{-0.25f, 0.0f}, {0.25f, 0.0f}, 0.1f};
    m2Polygon chip = m2MakeBox(0.25f, 0.25f);
    m2Transform pose = {above, {1.0f, 0.0f}};
    misuse = m2World_GetCounters(world).misuseCount;
    CHECK(!m2World_CastRayClosest(world, above, down, all, NULL) &&
              !m2World_CastCircleClosest(world, &ball, pose, down, all, NULL) &&
              !m2World_CastCapsuleClosest(world, &rod, pose, down, all, NULL) &&
              !m2World_CastPolygonClosest(world, &chip, pose, down, all, NULL) &&
              !m2Shape_CastRay(world, floor, above, down, NULL),
          "a null hitOut is refused");
    CHECK(m2World_GetCounters(world).misuseCount == misuse + CLOSEST_CASTS,
          "each counting one misuse");
    m2DestroyWorld(world);
}

enum
{
    LIST_CAPACITY = 3
};

// One array for each read that fills a caller list.
typedef struct ListArrays
{
    m2BodyId bodies[LIST_CAPACITY];
    m2JointId joints[LIST_CAPACITY];
    m2JointId bodyJoints[LIST_CAPACITY];
    m2ChainId chains[LIST_CAPACITY];
    m2ShapeId bodyShapes[LIST_CAPACITY];
    m2ShapeId chainShapes[LIST_CAPACITY];
    m2ShapeId sensed[LIST_CAPACITY];
    m2ParticleId particles[LIST_CAPACITY];
    m2ContactData contacts[LIST_CAPACITY];
    m2ShapeId overlaps[4][LIST_CAPACITY];
    m2CastHit hits[4][LIST_CAPACITY];
    m2MoverPlane planes[LIST_CAPACITY];
    m2ParticleId particlesInBox[LIST_CAPACITY];
} ListArrays;

// Runs every list read, the queries at `at`, each into its own array
// filled first with a pattern no read writes.
static void ReadEveryList(const m2World* world, m2BodyId body, m2ChainId chain, m2ShapeId sensor,
                          m2Pos2 at, ListArrays* a)
{
    memset(a, 0x5A, sizeof(*a));
    m2QueryFilter all = m2DefaultQueryFilter();
    m2Circle ball = {{0.0f, 0.0f}, 0.25f};
    m2Capsule rod = {{-0.25f, 0.0f}, {0.25f, 0.0f}, 0.1f};
    m2Polygon chip = m2MakeBox(0.25f, 0.25f);
    m2Transform pose = {at, {1.0f, 0.0f}};
    m2Vec2 down = {0.0f, -1.0f};
    m2WorldAabb box = {{at.x - 1.0, at.y - 1.0}, {at.x + 1.0, at.y + 1.0}};
    (void)m2World_GetBodies(world, a->bodies, LIST_CAPACITY);
    (void)m2World_GetJoints(world, a->joints, LIST_CAPACITY);
    (void)m2Body_GetJoints(world, body, a->bodyJoints, LIST_CAPACITY);
    (void)m2World_GetChains(world, a->chains, LIST_CAPACITY);
    (void)m2Body_GetShapes(world, body, a->bodyShapes, LIST_CAPACITY);
    (void)m2Chain_GetShapes(world, chain, a->chainShapes, LIST_CAPACITY);
    (void)m2Shape_GetSensorOverlaps(world, sensor, a->sensed, LIST_CAPACITY);
    (void)m2World_GetParticles(world, a->particles, LIST_CAPACITY);
    (void)m2World_GetContactData(world, a->contacts, LIST_CAPACITY);
    (void)m2World_OverlapAabb(world, box, all, a->overlaps[0], LIST_CAPACITY);
    (void)m2World_OverlapCircle(world, &ball, pose, all, a->overlaps[1], LIST_CAPACITY);
    (void)m2World_OverlapCapsule(world, &rod, pose, all, a->overlaps[2], LIST_CAPACITY);
    (void)m2World_OverlapPolygon(world, &chip, pose, all, a->overlaps[3], LIST_CAPACITY);
    (void)m2World_CastRayAll(world, at, down, all, a->hits[0], LIST_CAPACITY);
    (void)m2World_CastCircleAll(world, &ball, pose, down, all, a->hits[1], LIST_CAPACITY);
    (void)m2World_CastCapsuleAll(world, &rod, pose, down, all, a->hits[2], LIST_CAPACITY);
    (void)m2World_CastPolygonAll(world, &chip, pose, down, all, a->hits[3], LIST_CAPACITY);
    (void)m2World_CollideMover(world, &rod, pose, all, a->planes, LIST_CAPACITY);
    (void)m2World_OverlapParticlesAabb(world, box, a->particlesInBox, LIST_CAPACITY);
}

static bool Zeroed(const void* items, size_t bytes)
{
    const unsigned char* b = (const unsigned char*)items;
    for (size_t i = 0; i < bytes; ++i)
    {
        if (b[i] != 0)
        {
            return false;
        }
    }
    return true;
}

#define ZEROED(array) Zeroed((array), sizeof(array))

// How many of the stale id reads' arrays are zeroed.
static int32_t StaleListsZeroed(const ListArrays* a)
{
    return ZEROED(a->bodyJoints) + ZEROED(a->bodyShapes) + ZEROED(a->chainShapes) +
           ZEROED(a->sensed);
}

// How many of the queries' arrays are zeroed.
static int32_t QueryListsZeroed(const ListArrays* a)
{
    int32_t zeroed = ZEROED(a->planes) + ZEROED(a->particlesInBox);
    for (int32_t i = 0; i < 4; ++i)
    {
        zeroed += ZEROED(a->overlaps[i]) + ZEROED(a->hits[i]);
    }
    return zeroed;
}

// A refused list read defines none of its items, so it zeroes the caller's
// array over its whole capacity: an item from an earlier call is never
// read as an answer, and zeroed ids are null ids.
static void TestRefusedListsClear(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.particleCapacity = 4;
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.isSensor = true;
    m2Polygon slab = m2MakeBox(2.0f, 0.5f);
    m2ShapeId sensor = NewPolygonShape(world, body, &sd, &slab);
    m2Vec2 pts[4] = {{3.0f, 2.0f}, {1.0f, 2.0f}, {-1.0f, 2.0f}, {-3.0f, 2.0f}};
    m2ChainDef cd = m2DefaultChainDef();
    cd.points = pts;
    cd.count = 4;
    m2ChainId chain = NewChain(world, body, &cd);
    CHECK(chain.index1 != 0, "the chain is made");
    m2Pos2 above = {0.0, 3.0};
    ListArrays a;

    ReadEveryList(NULL, body, chain, sensor, above, &a);
    CHECK(ZEROED(a.bodies) && ZEROED(a.joints) && ZEROED(a.chains) && ZEROED(a.particles) &&
              ZEROED(a.contacts) && StaleListsZeroed(&a) == 4 && QueryListsZeroed(&a) == 10,
          "every list read on a null world zeroes its array");

    uint64_t misuse = m2World_GetCounters(world).misuseCount;
    m2BodyId staleBody = {body.index1, body.generation + 1u};
    m2ChainId staleChain = {chain.index1, chain.generation + 1u};
    m2ShapeId staleSensor = {sensor.index1, sensor.generation + 1u};
    ReadEveryList(world, staleBody, staleChain, staleSensor, above, &a);
    CHECK(StaleListsZeroed(&a) == 4, "a read with a stale id zeroes its array");
    CHECK(m2World_GetCounters(world).misuseCount == misuse + 4, "each counting one misuse");

    misuse = m2World_GetCounters(world).misuseCount;
    ReadEveryList(world, body, chain, sensor, (m2Pos2){(double)NAN, 3.0}, &a);
    CHECK(QueryListsZeroed(&a) == 10, "a refused query zeroes its array");
    CHECK(m2World_GetCounters(world).misuseCount == misuse + 10, "each counting one misuse");
    CHECK(StaleListsZeroed(&a) == 0 && !ZEROED(a.bodies),
          "and a read that succeeds writes only the items it defines");
    m2DestroyWorld(world);
}

int main(void)
{
    TestRayFromTheSurfaceHitsAtZero();
    TestFarAndLongRaysAtRoundShapes();
    TestCastHitOut();
    TestRefusedListsClear();
    TestRayInsideRoundedPolygon();
    TestAllHitsBeyondSixtyFour();
    TestQueryFilters();
    TestContactData();
    TestRayClosest();
    TestShapeCastsAndOverlaps();
    TestAllHits();
    TestMoverKit();
    TestRayGeometries();
    TestRayFarFromOrigin();
    TestOverlap();
    TestQueriesAreReadOnly();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
