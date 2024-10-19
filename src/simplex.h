#pragma once
#include<stdint.h>

/// @name 32 bit simplex noise functions
///@{
/// 32 bit, fixed point implementation of simplex noise functions.
/// The inputs are 20.12 fixed-point value. The result covers the full
/// range of a uint16_t averaging around 32768.
uint16_t snoise_1D(uint32_t x);
uint16_t snoise_2D(uint32_t x, uint32_t y);
uint16_t snoise_3D(uint32_t x, uint32_t y, uint32_t z);
uint16_t snoise_4D(uint32_t x, uint32_t y, uint32_t z, uint32_t w);
///@}

