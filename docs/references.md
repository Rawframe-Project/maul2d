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
