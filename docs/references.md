# References

Published sources behind the engines' algorithms. Code is written from
these descriptions, never copied from other implementations (see
[conventions.md](conventions.md), section 16).

## Elementary functions

- W. J. Cody and W. Waite, *Software Manual for the Elementary
  Functions*, Prentice-Hall, 1980. Argument reduction with a constant
  split into a high part exact in the working precision and a low
  correction; used for the reduction of angles to [-pi/4, pi/4] before
  the sine and cosine series.
- M. Abramowitz and I. A. Stegun, *Handbook of Mathematical Functions*,
  1964, sections 4.3 and 4.4: the Taylor series of sine, cosine and
  arctangent, and the addition formula for the arctangent that moves
  its argument toward zero (atan(a) = pi/6 + atan((sqrt(3) a - 1) /
  (a + sqrt(3)))).

## Bounding volume hierarchies

- J. Goldsmith and J. Salmon, "Automatic Creation of Object Hierarchies
  for Ray Tracing", *IEEE Computer Graphics and Applications* 7(5),
  1987. Incremental insertion into a box hierarchy guided by surface
  area, where the cost of placing a box next to a node is the area of
  the new parent plus the growth of every ancestor; used by the
  broadphase tree to pick a leaf's sibling.
- G. M. Adelson-Velsky and E. M. Landis, "An Algorithm for the
  Organization of Information", *Soviet Mathematics Doklady* 3, 1962.
  Height-balanced binary trees kept balanced by single and double
  rotations; the broadphase tree keeps this rule so its depth, and the
  query stack, stay bounded.
- D. E. Knuth, *The Art of Computer Programming*, Vol. 3, 2nd ed.,
  Addison-Wesley, 1998, section 6.2.3: concatenation of balanced trees,
  which the tree uses when a leaf is paired with a taller subtree.
- T. L. Kay and J. T. Kajiya, "Ray Tracing Complex Scenes", *SIGGRAPH
  1986*. Hierarchies built top down by splitting a set of objects along
  an axis; the maul3d tree rebuild splits at the median centroid along
  the widest axis.

## Contact and distance

- E. G. Gilbert, D. W. Johnson and S. S. Keerthi, "A Fast Procedure
  for Computing the Distance Between Complex Objects in
  Three-Dimensional Space", *IEEE Journal on Robotics and Automation*
  4(2), 1988. Distance between convex sets through a simplex in their
  Minkowski difference that shrinks toward the origin.
- G. van den Bergen, "A Fast and Robust GJK Implementation for
  Collision Detection of Convex Objects", *Journal of Graphics Tools*
  4(2), 1999. The relative stopping rule for the distance iteration.
- B. Mirtich, *Impulse-based Dynamic Simulation of Rigid Body
  Systems*, PhD thesis, University of California, Berkeley, 1996.
  Conservative advancement: a moving shape advances by its gap over its
  approach speed until it touches; used by the shape casts.
- I. E. Sutherland and G. W. Hodgman, "Reentrant Polygon Clipping",
  *Communications of the ACM* 17(1), 1974. Clipping against half
  planes; the polygon manifold clips the incident edge to the
  reference face's side planes.
- C. Ericson, *Real-Time Collision Detection*, Morgan Kaufmann, 2005:
  the separating axis test (section 4.4 and chapter 5), closest points
  between segments (5.1.9) and the segment against box test (5.3.3).
- M. Cyrus and J. Beck, "Generalized Two- and Three-Dimensional
  Clipping", *Computers and Graphics* 3(1), 1978. Clipping a line
  against a convex polygon face by face; used by the polygon ray cast.
- A. M. Andrew, "Another Efficient Algorithm for Convex Hulls in Two
  Dimensions", *Information Processing Letters* 9(5), 1979. The
  monotone chain hull.
- C. B. Barber, D. P. Dobkin and H. Huhdanpaa, "The Quickhull Algorithm
  for Convex Hulls", *ACM Transactions on Mathematical Software* 22(4),
  1996. The 3D hull: points wait on the face they lie farthest above,
  and the farthest is added next through the horizon of the faces it
  sees.
- D. Gregorius, "The Separating Axis Test between Convex Polyhedra",
  Game Developers Conference, 2013. Edge pairs of two hulls only form a
  face of their Minkowski difference when the Gauss map arcs of their
  adjacent face normals cross; only those pairs are tested.
- M. E. Newell's method for the plane of a polygon (see I. E.
  Sutherland, R. F. Sproull and R. A. Schumacker, "A Characterization of
  Ten Hidden-Surface Algorithms", *Computing Surveys* 6(1), 1974): the
  normal as the sum over edges, robust for nearly flat loops.

## Mass properties

- J. Blow and A. J. Binstock, "How to Find the Inertia Tensor (or Other
  Mass Properties) of a 3D Solid Body Represented by a Triangle Mesh",
  2004. The covariance of a tetrahedron as the canonical tetrahedron's
  covariance mapped by the tetrahedron's edge matrix; the 3D hull mass
  sums it over a fan of each face.
- J. Steiner's formula for parallel bodies: a convex region swollen by
  a radius r gains its perimeter times r plus a disc of radius r. The
  mass of rounded polygons and capsules follows this decomposition
  into the core, one rectangle per edge and one circular sector per
  corner, each part's centroid and second moment taken in closed form
  (for example from the tables in F. P. Beer and E. R. Johnston,
  *Vector Mechanics for Engineers: Statics*).
