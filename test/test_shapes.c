// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shape checks: validation thresholds (the relative-threshold
// family, adversarial inputs included), rotation-aware AABBs against
// analytic expectations, mass properties against closed forms, mixed
// shapes through the pair pipeline, rollback over shape state, and the
// shape sweep hash compared across CI cells.

#include "helpers.h"
#include "test_harness.h"

#include "maul2d/base.h"
#include "maul2d/maul2d.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK_NEAR(value, expected, tolerance, msg)                                                \
    CHECK((value) > (expected) - (tolerance) && (value) < (expected) + (tolerance), msg)

static void TestValidation(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();

    // Degenerate capsule: points one ulp apart must be rejected (the axis
    // normalization would divide by ~1e-38).
    m2Capsule tiny = {{1.0f, 0.0f}, {1.0000001f, 0.0f}, 0.3f};
    CHECK(NewCapsuleShape(world, body, &sd, &tiny).index1 == 0, "1-ulp capsule must be rejected");
    m2Capsule good = {{-0.5f, 0.0f}, {0.5f, 0.0f}, 0.3f};
    CHECK(m2Shape_IsValid(world, NewCapsuleShape(world, body, &sd, &good)),
          "ordinary capsule must pass");

    // Sliver polygon: large area but pathological thinness (scale-free
    // rejection; an absolute epsilon would pass this one).
    m2Vec2 sliver[3] = {{0.0f, 0.0f}, {100.0f, 0.0f}, {100.0f, 0.002f}};
    m2Polygon sliverPoly = MakePolygon(sliver, 3, 0.0f);
    CHECK(sliverPoly.count == 0, "sliver polygon must be rejected");

    // Winding: clockwise input violates the CCW contract - rejected, not
    // silently repaired.
    m2Vec2 clockwise[3] = {{0.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 0.0f}};
    CHECK(MakePolygon(clockwise, 3, 0.0f).count == 0, "CW winding must be rejected");

    // NaN screening at the rim.
    uint32_t nanBits = 0x7FC00000u;
    float qnan;
    memcpy(&qnan, &nanBits, sizeof(qnan));
    m2Circle nanCircle = {{qnan, 0.0f}, 0.5f};
    CHECK(NewCircleShape(world, body, &sd, &nanCircle).index1 == 0,
          "NaN geometry must be rejected");

    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    CHECK(box.count == 4 && m2Shape_IsValid(world, NewPolygonShape(world, body, &sd, &box)),
          "unit box must validate");
    m2DestroyWorld(world);
}

static void TestAabbAndMass(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 16;
    m2World* worldId = NewWorld(&def);

    // A body rotated 90 degrees with an offset circle: the AABB must
    // follow the rotation (the slice-1 placeholder ignored it).
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){10.0, 20.0};
    bd.rotation = m2MakeRot(0.5f * M2_PI);
    m2BodyId body = NewBody(worldId, &bd);

    m2ShapeDef sd = m2DefaultShapeDef();
    sd.density = 2.0f;
    m2Circle circle = {{3.0f, 0.0f}, 0.5f};
    m2ShapeId shape = NewCircleShape(worldId, body, &sd, &circle);
    CHECK(m2Shape_IsValid(worldId, shape), "circle attaches");

    // Rotating (3,0) by +90deg lands at (0,3): center ~(10, 23).
    m2WorldAabb tight = m2Shape_GetAabb(worldId, shape);
    CHECK_NEAR(tight.lowerBound.x, 10.0 - 0.5, 1.0e-3, "rotated AABB lower x");
    CHECK_NEAR(tight.upperBound.y, 23.0 + 0.5, 1.0e-3, "rotated AABB upper y");

    // Mass: circle m = rho * pi * r^2.
    CHECK_NEAR(m2Body_GetMass(worldId, body), 2.0f * 3.14159f * 0.25f, 1.0e-3f, "circle mass");

    // Box inertia about origin for a centered box: m*(w^2+h^2)/12.
    m2BodyDef bd2 = m2DefaultBodyDef();
    bd2.type = m2_dynamicBody;
    m2BodyId boxBody = NewBody(worldId, &bd2);
    m2ShapeDef sd2 = m2DefaultShapeDef();
    sd2.density = 1.0f;
    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    NewPolygonShape(worldId, boxBody, &sd2, &box);
    CHECK_NEAR(m2Body_GetMass(worldId, boxBody), 1.0f, 1.0e-4f, "unit box mass");
    float expectedInvI = 1.0f / (1.0f * (1.0f + 1.0f) / 12.0f);
    CHECK_NEAR(1.0f / m2Body_GetCentralInertia(worldId, boxBody), expectedInvI, 1.0e-2f,
               "box inertia");

    // Zero-density dynamic body: the minimum-mass floor, never NaN.
    m2BodyDef bd3 = m2DefaultBodyDef();
    bd3.type = m2_dynamicBody;
    m2BodyId floored = NewBody(worldId, &bd3);
    m2ShapeDef sd3 = m2DefaultShapeDef();
    sd3.density = 0.0f;
    NewCircleShape(worldId, floored, &sd3, &circle);
    CHECK(m2Body_GetMass(worldId, floored) == 1.0f, "zero-density dynamic body floors to 1 kg");

    // Inertia accuracy far from the origin. A single circle of radius 1 at
    // local (1000,0) has its COM at its own center, so its inertia ABOUT
    // the COM is exactly m*0.5*r^2, independent of the offset. The old
    // origin-then-subtract math lost ~1000^2 * epsilon to cancellation here
    // (measured error ~7e-2, and it hits zero entirely by an offset of
    // 5000); the centroid-based accumulation holds it
    // at float epsilon. The tight tolerance is the test: it passes with the
    // centroid math and fails with the cancellation.
    m2BodyDef bd4 = m2DefaultBodyDef();
    bd4.type = m2_dynamicBody;
    m2BodyId farBody = NewBody(worldId, &bd4);
    m2ShapeDef sd4 = m2DefaultShapeDef();
    sd4.density = 1.0f;
    m2Circle farCircle = {{1000.0f, 0.0f}, 1.0f};
    NewCircleShape(worldId, farBody, &sd4, &farCircle);
    float farMass = m2Body_GetMass(worldId, farBody);
    CHECK_NEAR(m2Body_GetCentralInertia(worldId, farBody), farMass * 0.5f, 1.0e-3f,
               "a far-off-origin circle's inertia is cancellation-free");

    // Two circles straddling the origin: the COM lands between them, so the
    // parallel-axis offsets are real (distance 3 each) and the accumulation
    // must match the analytic sum 2*(m*0.5*r^2 + m*3^2).
    m2BodyId two = NewBody(worldId, &bd4);
    m2Circle left = {{-3.0f, 0.0f}, 0.5f};
    m2Circle right = {{3.0f, 0.0f}, 0.5f};
    NewCircleShape(worldId, two, &sd4, &left);
    NewCircleShape(worldId, two, &sd4, &right);
    float mc = 3.14159f * 0.25f; // each circle mass, r=0.5
    float twoExpected = 2.0f * (mc * 0.5f * 0.25f + mc * 9.0f);
    CHECK_NEAR(m2Body_GetCentralInertia(worldId, two), twoExpected, 1.0e-2f,
               "a symmetric two-circle body matches the analytic inertia");

    m2DestroyWorld(worldId);
}

// Editor round-trip: what you created is what you read back, to the
// exact stored bit, for every geometry kind plus the body state that
// gizmos draw from.
static void TestGeometryReadback(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 32;
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 4.0};
    bd.enableBullet = true;
    bd.gravityScale = 0.5f;
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.density = 2.5f;

    m2Circle circle = {{1.0f, 0.25f}, 0.4f};
    m2ShapeId circleShape = NewCircleShape(world, body, &sd, &circle);
    m2Capsule capsule = {{-0.5f, 0.0f}, {0.5f, 0.0f}, 0.3f};
    m2ShapeId capsuleShape = NewCapsuleShape(world, body, &sd, &capsule);
    m2Polygon poly = m2MakeBox(0.6f, 0.2f);
    m2ShapeId polyShape = NewPolygonShape(world, body, &sd, &poly);

    m2Circle rc = m2CircleShape_GetCircle(world, circleShape);
    CHECK(rc.center.x == circle.center.x && rc.center.y == circle.center.y &&
              rc.radius == circle.radius,
          "circle reads back exactly");
    m2Capsule rcap = m2CapsuleShape_GetCapsule(world, capsuleShape);
    CHECK(rcap.point1.x == capsule.point1.x && rcap.point2.x == capsule.point2.x &&
              rcap.radius == capsule.radius,
          "capsule reads back exactly");
    m2Polygon rp = m2PolygonShape_GetPolygon(world, polyShape);
    bool polySame = rp.count == poly.count;
    for (int32_t i = 0; polySame && i < rp.count; ++i)
    {
        polySame = rp.vertices[i].x == poly.vertices[i].x && rp.vertices[i].y == poly.vertices[i].y;
    }
    CHECK(polySame, "polygon vertices read back exactly");
    CHECK(m2Shape_GetDensity(world, polyShape) == 2.5f, "density reads back");

    // Body state for gizmos: the compound is symmetric about x != 0,
    // so the local center must sit off the origin, and the def flags
    // read back as set.
    m2Vec2 com = m2Body_GetLocalCenter(world, body);
    CHECK(com.x != 0.0f, "offset shapes move the local center");
    CHECK(m2Body_IsBulletEnabled(world, body), "bullet flag reads back");
    CHECK(m2Body_GetGravityScale(world, body) == 0.5f, "gravity scale reads back");

    // Chain links: enumerable per chain, each one a chain segment,
    // and the middle link's stored points echo the source polyline.
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &gd);
    m2Vec2 pts[6] = {{6.0f, 0.0f}, {4.0f, 0.0f},  {2.0f, 0.0f},
                     {0.0f, 0.0f}, {-2.0f, 0.0f}, {-4.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 6;
    m2ChainId ledge = NewChain(world, ground, &chain);
    CHECK(m2Chain_GetShapes(world, ledge, NULL, 0) == m2Chain_GetSegmentCount(world, ledge),
          "chain shape walk agrees with the segment count");
    m2ShapeId links[8];
    int32_t linkCount = m2Chain_GetShapes(world, ledge, links, 8);
    CHECK(linkCount == 3, "three links");
    bool allSegments = true;
    bool ascending = true;
    for (int32_t i = 0; i < linkCount; ++i)
    {
        allSegments = allSegments && m2Shape_GetType(world, links[i]) == m2_chainSegmentShape;
        ascending = ascending && (i == 0 || links[i - 1].index1 < links[i].index1);
    }
    CHECK(allSegments && ascending, "all links are segments, ascending");
    m2ChainSegment mid = m2ChainSegmentShape_GetChainSegment(world, links[1]);
    CHECK(mid.segment.point1.x == 2.0f && mid.segment.point2.x == 0.0f && mid.ghost1.x == 4.0f &&
              mid.ghost2.x == -2.0f,
          "the middle link's points echo the polyline");

    m2DestroyWorld(world);
}

// m2ComputeHull: welding, collinear rejection, and a noisy cloud
// collapsing to the square it always was.
static float PieceArea(const m2Polygon* p)
{
    float area2 = 0.0f;
    for (int32_t i = 0; i < p->count; ++i)
    {
        m2Vec2 a = p->vertices[i];
        m2Vec2 b = p->vertices[(i + 1) % p->count];
        area2 += a.x * b.y - b.x * a.y;
    }
    return 0.5f * area2;
}

// The sprite-outline road: concave outlines split into convex pieces
// that keep the area, stay inside the 8-vertex law, and validate
// through the ordinary polygon road. Bad outlines bounce loudly.
static void TestDecomposeOutline(void)
{
    // An L: one reflex corner, two pieces, area 4.
    m2Vec2 ell[6] = {{0.0f, 0.0f}, {2.0f, 0.0f}, {2.0f, 1.0f},
                     {1.0f, 1.0f}, {1.0f, 3.0f}, {0.0f, 3.0f}};
    m2Polygon pieces[16];
    int32_t n = DecomposeOutline(ell, 6, pieces, 16);
    CHECK(n == 2, "an L splits into two convex pieces");
    float area = 0.0f;
    for (int32_t i = 0; i < n; ++i)
    {
        CHECK(pieces[i].count >= 3 && pieces[i].count <= 8, "pieces obey the vertex law");
        area += PieceArea(&pieces[i]);
    }
    CHECK(area > 3.999f && area < 4.001f, "the L keeps its area");

    // A plus: four reflex corners, area 20, a handful of pieces.
    m2Vec2 plus[12] = {{3.0f, -1.0f},  {3.0f, 1.0f},   {1.0f, 1.0f},  {1.0f, 3.0f},
                       {-1.0f, 3.0f},  {-1.0f, 1.0f},  {-3.0f, 1.0f}, {-3.0f, -1.0f},
                       {-1.0f, -1.0f}, {-1.0f, -3.0f}, {1.0f, -3.0f}, {1.0f, -1.0f}};
    n = DecomposeOutline(plus, 12, pieces, 16);
    CHECK(n >= 3 && n <= 5, "a plus needs a handful of pieces");
    area = 0.0f;
    for (int32_t i = 0; i < n; ++i)
    {
        area += PieceArea(&pieces[i]);
    }
    CHECK(area > 19.99f && area < 20.01f, "the plus keeps its area");

    // Truthful totals beyond capacity, the enumeration contract.
    m2Polygon one[1];
    int32_t total = DecomposeOutline(plus, 12, one, 1);
    CHECK(total == n, "capacity pressure never lies about the total");

    // A convex outline is returned whole.
    m2Vec2 box[4] = {{0.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 1.0f}};
    n = DecomposeOutline(box, 4, pieces, 16);
    CHECK(n == 1 && pieces[0].count == 4, "a convex outline stays one piece");

    // The pieces build real bodies end to end.
    m2WorldDef wd = m2DefaultWorldDef();
    wd.bodyCapacity = 8;
    wd.shapeCapacity = 16;
    wd.jointCapacity = 4;
    m2World* world = NewWorld(&wd);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 5.0};
    m2BodyId body = NewBody(world, &bd);
    int32_t made = DecomposeOutline(ell, 6, pieces, 16);
    for (int32_t i = 0; i < made; ++i)
    {
        m2ShapeDef sd = m2DefaultShapeDef();
        m2ShapeId sid = NewPolygonShape(world, body, &sd, &pieces[i]);
        CHECK(m2Shape_IsValid(world, sid), "every piece becomes a live shape");
    }
    m2DestroyWorld(world);
}

static void TestComputeHull(void)
{
    // A square with duplicates, an interior point, and jitter on one
    // edge that should weld or merge away.
    m2Vec2 cloud[8] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f},  {-1.0f, 1.0f},
                       {0.0f, 0.2f},   {1.0f, -1.0f}, {0.0f, -1.0f}, {-1.0f, -0.999f}};
    m2Polygon hull = ComputeHull(cloud, 8, 0.0f);
    CHECK(hull.count == 4, "the noisy cloud is a square");
    float loX = 3.4e38f;
    float hiX = -3.4e38f;
    for (int32_t i = 0; i < hull.count; ++i)
    {
        loX = hull.vertices[i].x < loX ? hull.vertices[i].x : loX;
        hiX = hull.vertices[i].x > hiX ? hull.vertices[i].x : hiX;
    }
    CHECK(loX == -1.0f && hiX == 1.0f, "with the original extremes");

    // The hull feeds shape creation directly.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 5.0};
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2ShapeId shape = NewPolygonShape(world, body, &sd, &hull);
    CHECK(m2Shape_IsValid(world, shape), "and the world accepts it");
    m2DestroyWorld(world);

    // Degenerates are loud, not quiet.
    m2Vec2 line[4] = {{0.0f, 0.0f}, {1.0f, 0.0f}, {2.0f, 0.0f}, {3.0f, 0.0f}};
    CHECK(ComputeHull(line, 4, 0.0f).count == 0, "collinear input returns count zero");
    m2Vec2 blob[3] = {{0.0f, 0.0f}, {0.001f, 0.001f}, {0.002f, 0.0f}};
    CHECK(ComputeHull(blob, 3, 0.0f).count == 0, "a welded-away blob returns count zero");

    // The whole cloud makes the hull: a point past the eighth is in it.
    m2Vec2 spur[9] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f},  {-1.0f, 1.0f}, {0.0f, -1.0f},
                      {1.0f, 0.0f},   {0.0f, 1.0f},  {-1.0f, 0.0f}, {5.0f, 0.0f}};
    hull = ComputeHull(spur, 9, 0.0f);
    bool reachesSpur = false;
    for (int32_t i = 0; i < hull.count; ++i)
    {
        reachesSpur = reachesSpur || (hull.vertices[i].x == 5.0f && hull.vertices[i].y == 0.0f);
    }
    CHECK(hull.count == 5 && reachesSpur, "the ninth point of a cloud is in its hull");

    // A hull of more vertices than a polygon holds, and a cloud past
    // M2_MAX_HULL_POINTS, are past a bound.
    m2Vec2 ring[M2_MAX_HULL_POINTS + 1];
    for (int32_t i = 0; i <= M2_MAX_HULL_POINTS; ++i)
    {
        m2Rot turn = m2MakeRot(2.0f * M2_PI * (float)(i % 12) / 12.0f);
        ring[i] = (m2Vec2){3.0f * turn.c, 3.0f * turn.s};
    }
    m2Polygon kept = m2MakeBox(1.0f, 1.0f);
    CHECK(m2ComputeHull(ring, 12, 0.0f, &kept) == m2_errorLimit && kept.count == 4,
          "a hull of 12 vertices is m2_errorLimit and leaves the output as it was");
    CHECK(m2ComputeHull(ring, M2_MAX_HULL_POINTS + 1, 0.0f, &kept) == m2_errorLimit,
          "a cloud past M2_MAX_HULL_POINTS is m2_errorLimit");
    CHECK(ComputeHull(ring, 8, 0.0f).count == 8, "the first 8 of the ring are its own hull");
}

// Mass properties against closed forms: a capsule is a rectangle and
// two half discs whose centroids sit 4r / (3 pi) past the segment ends,
// and a rounded square adds edge rectangles and corner quarter discs to
// its core (Steiner's decomposition).
static void TestMassClosedForms(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.density = 1.0f;

    m2BodyId pill = NewBody(world, &bd);
    m2Capsule capsule = {{-1.0f, 0.0f}, {1.0f, 0.0f}, 0.5f};
    NewCapsuleShape(world, pill, &sd, &capsule);
    double disc = 3.14159265358979 * 0.25;
    double lc = 4.0 * 0.5 / (3.0 * 3.14159265358979);
    double capsuleInertia = 2.0 * (4.0 + 1.0) / 12.0 + disc * (0.5 * 0.25 + 1.0 + 2.0 * lc);
    CHECK(fabs((double)m2Body_GetMass(world, pill) - (2.0 + disc)) < 1.0e-4, "capsule mass");
    CHECK(fabs((double)m2Body_GetCentralInertia(world, pill) - capsuleInertia) < 1.0e-4,
          "capsule inertia");

    m2BodyId block = NewBody(world, &bd);
    m2Vec2 square[4] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}};
    m2Polygon rounded = MakePolygon(square, 4, 0.5f);
    NewPolygonShape(world, block, &sd, &rounded);
    double area = 4.0 + 8.0 * 0.5 + disc;
    double core = 4.0 * 8.0 / 12.0;
    double rects = 4.0 * (1.0 * (4.0 + 0.25) / 12.0 + 1.25 * 1.25);
    double quarter = disc / 4.0;
    double reach = 2.0 * 0.5 * 0.70710678118655 / (3.0 * 3.14159265358979 / 4.0);
    double out = 1.41421356237310 + reach;
    double corners = 4.0 * (quarter * (0.5 * 0.25 - reach * reach) + quarter * out * out);
    CHECK(fabs((double)m2Body_GetMass(world, block) - area) < 1.0e-4, "rounded square mass");
    CHECK(fabs((double)m2Body_GetCentralInertia(world, block) - (core + rects + corners)) < 1.0e-3,
          "rounded square inertia");
    m2DestroyWorld(world);
}

// A world keeps a polygon's vertices, count and radius and computes the
// rest: normals the caller filled in (here NaN) and junk in the unused
// slots are not kept. fuzz_snapshot found polygons with bad normals
// reaching an assert in the mass computation through a snapshot; the API
// took them as given too.
static void TestPolygonNormalsAreTheWorlds(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    m2BodyDef bodyDef = m2DefaultBodyDef();
    bodyDef.type = m2_dynamicBody;
    m2BodyId body = NewBody(world, &bodyDef);
    m2Polygon built = m2MakeBox(0.5f, 0.25f);
    m2Polygon given = built;
    for (int32_t i = 0; i < M2_MAX_POLYGON_VERTICES; ++i)
    {
        given.normals[i] = (m2Vec2){NAN, 7.0f};
    }
    given.vertices[M2_MAX_POLYGON_VERTICES - 1] = (m2Vec2){INFINITY, 3.0f};
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    m2ShapeId shape = m2_nullShapeId;
    CHECK(m2CreatePolygonShape(world, body, &shapeDef, &given, &shape) == m2_success,
          "a polygon with the caller's normals is created");
    m2Polygon kept = m2PolygonShape_GetPolygon(world, shape);
    CHECK(memcmp(&kept, &built, sizeof(kept)) == 0, "the world keeps its own normals");
    m2Shape_SetPolygon(world, shape, &given);
    kept = m2PolygonShape_GetPolygon(world, shape);
    CHECK(memcmp(&kept, &built, sizeof(kept)) == 0, "and keeps them when the polygon is set");
    m2DestroyWorld(world);
}

// Body-local lengths stop at M2_LOCAL_LIMIT: past it the solver's float products
// overflow. Each create and builder refuses, and a refusal counts one
// misuse where a world is involved.
static void TestLocalLengthsAreBounded(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    m2BodyDef bodyDef = m2DefaultBodyDef();
    bodyDef.type = m2_dynamicBody;
    m2BodyId body = NewBody(world, &bodyDef);
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    const float beyond = 2.0f * M2_LOCAL_LIMIT;
    m2ShapeId shape = m2_nullShapeId;

    m2Circle edge = {{0.0f, 0.0f}, M2_LOCAL_LIMIT};
    CHECK(m2CreateCircleShape(world, body, &shapeDef, &edge, &shape) == m2_success,
          "a circle of radius M2_LOCAL_LIMIT is created");
    uint64_t misuse = m2World_GetCounters(world).misuseCount;
    m2Circle wide = {{0.0f, 0.0f}, beyond};
    m2Circle offset = {{beyond, 0.0f}, 1.0f};
    m2Capsule capsule = {{0.0f, 0.0f}, {beyond, 0.0f}, 0.5f};
    m2Segment segment = {{0.0f, -beyond}, {0.0f, 1.0f}};
    m2Polygon box = m2MakeBox(beyond, beyond);
    CHECK(box.count == 0, "m2MakeBox refuses a box past M2_LOCAL_LIMIT");
    m2Polygon given = m2MakeBox(1.0f, 1.0f);
    given.vertices[2] = (m2Vec2){beyond, beyond};
    CHECK(m2CreateCircleShape(world, body, &shapeDef, &wide, &shape) == m2_errorLimit &&
              m2CreateCircleShape(world, body, &shapeDef, &offset, &shape) == m2_errorLimit &&
              m2CreateCapsuleShape(world, body, &shapeDef, &capsule, &shape) == m2_errorLimit &&
              m2CreateSegmentShape(world, body, &shapeDef, &segment, &shape) == m2_errorLimit &&
              m2CreatePolygonShape(world, body, &shapeDef, &given, &shape) == m2_errorLimit,
          "every shape past M2_LOCAL_LIMIT is refused");
    CHECK(m2World_GetCounters(world).misuseCount == misuse + 5, "each refusal counts one misuse");

    m2Vec2 points[4] = {{-1.0f, 0.0f}, {0.0f, 0.0f}, {beyond, 0.0f}, {beyond + 1.0f, 0.0f}};
    m2ChainDef chainDef = m2DefaultChainDef();
    chainDef.points = points;
    chainDef.count = 4;
    m2ChainId chain = m2_nullChainId;
    CHECK(m2CreateChain(world, body, &chainDef, &chain) == m2_errorLimit,
          "a chain point past M2_LOCAL_LIMIT is refused");

    m2Vec2 square[4] = {{0.0f, 0.0f}, {beyond, 0.0f}, {beyond, beyond}, {0.0f, beyond}};
    m2Polygon pieces[4];
    int32_t count = 7;
    CHECK(m2MakePolygon(square, 4, 0.0f, &given) == m2_errorLimit &&
              m2ComputeHull(square, 4, 0.0f, &given) == m2_errorLimit &&
              m2DecomposeOutline(square, 4, pieces, 4, &count) == m2_errorLimit && count == 7,
          "the builders refuse points past M2_LOCAL_LIMIT");
    m2DestroyWorld(world);
}

// A spike from the top edge whose tip stops 1 mm above the bottom edge: no
// edges meet, but the tip touches the bottom for the solver, so the
// outline is refused as not simple.
static void TestOutlineThatNearlyTouchesIsRefused(void)
{
    m2Vec2 spike[7] = {{0.0f, 0.0f},   {4.0f, 0.0f}, {4.0f, 2.0f}, {2.1f, 2.0f},
                       {2.0f, 0.001f}, {1.9f, 2.0f}, {0.0f, 2.0f}};
    m2Polygon pieces[8];
    int32_t count = 7;
    CHECK(m2DecomposeOutline(spike, 7, pieces, 8, &count) == m2_errorInvalid && count == 7,
          "a vertex within the slop of another edge is refused");
    spike[4].y = 0.01f;
    CHECK(m2DecomposeOutline(spike, 7, pieces, 8, &count) == m2_success && count > 1,
          "one twice the slop away decomposes");

    // A simple outline whose one piece fails validation is refused after
    // the decomposition ran: its pieces are left as they were.
    m2Vec2 sliver[3] = {{0.0f, 0.0f}, {100.0f, 0.0f}, {100.0f, 0.02f}};
    pieces[0].count = 99;
    count = 7;
    CHECK(m2DecomposeOutline(sliver, 3, pieces, 8, &count) == m2_errorInvalid && count == 7 &&
              pieces[0].count == 99,
          "an outline that keeps no piece leaves the pieces and the count as they were");
}

// A small polygon far from the origin of its body is valid. Its area was
// summed in float, where the cross products near 65536 m lie 512 apart, so
// a 1 m box there came out with an area of noise and was refused.
static void TestSmallPolygonFarFromOriginIsValid(void)
{
    const float o = 65536.0f;
    m2Vec2 box[4] = {
        {o - 0.5f, o - 0.5f}, {o + 0.5f, o - 0.5f}, {o + 0.5f, o + 0.5f}, {o - 0.5f, o + 0.5f}};
    m2Vec2 corner[3] = {{o, o}, {o + 1.0f, o}, {o, o + 1.0f}};
    m2Polygon polygon;
    CHECK(m2MakePolygon(box, 4, 0.0f, &polygon) == m2_success,
          "a 1 m box 65536 m from the origin is a polygon");
    CHECK(m2MakePolygon(corner, 3, 0.0f, &polygon) == m2_success, "and so is a 1 m triangle there");
}

// Outlines the decomposition used to accept. A small clockwise hexagon far
// from the origin: summed in float, its area came out positive, and it
// split into two pieces; wound the other way, it still decomposes. A long
// thin triangle: counter-clockwise, but too thin for any piece to pass
// validation, so it was accepted with none.
static void TestOutlineWithoutPiecesIsRefused(void)
{
    const float o = 98304.0f; // 1.5 * 2^16, where floats lie 1/128 m apart
    m2Vec2 clockwise[6] = {{o + 4.9609375f, o + 0.1875f},    {o + 2.84375f, o - 0.046875f},
                           {o - 1.2109375f, o - 2.3515625f}, {o - 1.25f, o - 0.0546875f},
                           {o - 1.59375f, o + 2.7265625f},   {o + 0.640625f, o + 1.1796875f}};
    m2Vec2 needle[3] = {{-5.0e4f, 0.0f}, {5.0e4f, 0.0f}, {0.0f, 0.01f}};
    m2Polygon pieces[4];
    int32_t count = 7;
    CHECK(m2DecomposeOutline(clockwise, 6, pieces, 4, &count) == m2_errorInvalid && count == 7,
          "a clockwise outline far from the origin is refused");
    m2Vec2 reversed[6];
    for (int32_t i = 0; i < 6; ++i)
    {
        reversed[i] = clockwise[5 - i];
    }
    int32_t made = 0;
    CHECK(m2DecomposeOutline(reversed, 6, pieces, 4, &made) == m2_success && made > 0,
          "the same outline wound counter-clockwise decomposes");
    CHECK(m2DecomposeOutline(needle, 3, pieces, 4, &count) == m2_errorInvalid && count == 7,
          "an outline that leaves no piece is refused");
    CHECK(m2DecomposeOutline(needle, 3, NULL, 0, &count) == m2_errorInvalid && count == 7,
          "and so is its measure");
}

// A static box that grows after the world has stepped: a ball dropped over
// the new part rests on it. The step refits only awake bodies' boxes, so
// the setter must refit the static shape's own box, or the broadphase
// keeps the old extent and the ball falls through.
static void TestGrownStaticShapeCollides(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    m2BodyDef groundDef = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &groundDef);
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    m2Polygon small = m2MakeBox(0.5f, 0.5f);
    m2ShapeId shape = NewPolygonShape(world, ground, &shapeDef, &small);
    m2World_Step(world, 1.0f / 60.0f, 4);
    m2Polygon big = m2MakeBox(10.0f, 0.5f);
    m2Shape_SetPolygon(world, shape, &big);
    m2BodyDef ballDef = m2DefaultBodyDef();
    ballDef.type = m2_dynamicBody;
    ballDef.position = (m2Pos2){8.0, 2.0};
    m2BodyId ball = NewBody(world, &ballDef);
    m2Circle circle = {{0.0f, 0.0f}, 0.25f};
    NewCircleShape(world, ball, &shapeDef, &circle);
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double y = m2Body_GetPosition(world, ball).y;
    CHECK(y > 0.7 && y < 0.8, "a ball dropped on a grown static box rests on it");
    m2DestroyWorld(world);
}

int main(void)
{
    TestGrownStaticShapeCollides();
    TestLocalLengthsAreBounded();
    TestOutlineThatNearlyTouchesIsRefused();
    TestSmallPolygonFarFromOriginIsValid();
    TestOutlineWithoutPiecesIsRefused();
    TestPolygonNormalsAreTheWorlds();
    TestMassClosedForms();
    TestValidation();
    TestComputeHull();
    TestDecomposeOutline();
    TestGeometryReadback();
    TestAabbAndMass();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
