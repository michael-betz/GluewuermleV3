#ifndef PERLIN_HEADER
#define PERLIN_HEADER
// source: https://github.com/czinn/perlin

float rawnoise(int n);

float noise1d(int x, int octave, int seed);

float noise2d(int x, int y, int octave, int seed);

float noise3d(int x, int y, int z, int octave, int seed);

float interpolate(float a, float b, float x);

float smooth1d(float x, int octave, int seed);

float smooth2d(float x, float y, int octave, int seed);

float smooth3d(float x, float y, float z, int octave, int seed);

float pnoise1d(float x, float persistence, int octaves, int seed);

float pnoise2d(float x, float y, float persistence, int octaves, int seed);

float pnoise3d(float x, float y, float z, float persistence, int octaves, int seed);

#endif
