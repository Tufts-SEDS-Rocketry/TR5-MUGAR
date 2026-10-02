#pragma once

#include <array>
#include <cmath>
#include <cstdint>

namespace seds {

float dot(std::array<float, 3> v1, std::array<float, 3> v2);

float mag(std::array<float, 3> vec);

std::array<float, 3> norm(std::array<float, 3> vec);

std::array<float, 4> norm4(std::array<float, 4> q);

// https://en.wikipedia.org/wiki/Cross_product#Computing
std::array<float, 3> cross(std::array<float, 3> v1, std::array<float, 3> v2);

std::array<float, 4> quat_mult(std::array<float, 4> q1, std::array<float, 4> q2);

// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Vector_rotation
std::array<float, 4> quat_rot(std::array<float, 3> omega, std::array<float, 4> q, float dt);

std::array<float, 3> rot_vec(std::array<float, 3> v, std::array<float, 4> q);

}