# Third-party notices

The engine contains no code from other projects. The published work
its algorithms follow is listed in [docs/references.md](docs/references.md).

## Testbed-only dependencies

The interactive testbed (never the engine) fetches
[raylib](https://github.com/raysan5/raylib) (zlib/libpng license)
at configure time when built with `-DMAUL2D_BUILD_TESTBED=ON`. The
engine library links against nothing.
