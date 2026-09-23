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
