#include "rotation.h"

#include "esp_log.h"

namespace seds {

float dot(std::array<float, 3> v1, std::array<float, 3> v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

float dot4(std::array<float, 4> v1, std::array<float, 4> v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2] + v1[3] * v2[3];
}

float mag(std::array<float, 3> vec) {
    return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

float mag4(std::array<float, 4> q) {
    return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

std::array<float, 3> norm(std::array<float, 3> vec) {
    float m = mag(vec);
    if (m == 0) {
        return vec;
    }
    return std::array<float, 3> { vec[0] / m, vec[1] / m, vec[2] / m };
}

std::array<float, 4> norm4(std::array<float, 4> q) {
    float m = mag4(q);
    if (m == 0) {
        return q;
    }

    return std::array<float, 4> { q[0] / m, q[1] / m, q[2] / m, q[3] / m };
}

// https://en.wikipedia.org/wiki/Cross_product#Computing
std::array<float, 3> cross(std::array<float, 3> v1, std::array<float, 3> v2) {
    float s1 = v1[1] * v2[2] - v1[2] * v2[1];
    float s2 = v1[2] * v2[0] - v1[0] * v2[2];
    float s3 = v1[0] * v2[1] - v1[1] * v2[0];
    
    return std::array<float, 3> { s1, s2, s3 };
}

std::array<float, 4> quat_mult(std::array<float, 4> q1, std::array<float, 4> q2) {
    std::array<float, 4> new_quat = {
        q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
        q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
        q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
        q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    };

    return new_quat;
}

// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Vector_rotation
// https://ahrs.readthedocs.io/en/latest/filters/angular.html
std::array<float, 4> quat_rot(std::array<float, 3> omega, std::array<float, 4> q, float dt) {
    float mag_om = mag(omega);
    float ident = std::cosf(mag_om * dt / 2);
    float omega_mat = (1 / mag_om) * std::sinf(mag_om * dt / 2);

    // check this!
    std::array<float, 4> rot = {
        ident, omega_mat * omega[0], omega_mat * omega[1], omega_mat * omega[2]
    };

    return quat_mult(rot, q);
}

// https://www.mathworks.com/help/fusion/ug/rotations-orientation-and-quaternions.html#d126e10045
std::array<float, 3> rot_vec(std::array<float, 3> v, std::array<float, 4> q) {
    std::array<float, 4> v_quat = { 0, v[0], v[1], v[2] };
    std::array<float, 4> q_conj = { q[0], -q[1], -q[2], -q[3] };
    std::array<float, 4> new_vq = quat_mult(quat_mult(q, v_quat), q_conj);
    return std::array<float, 3> { new_vq[1], new_vq[2], new_vq[3] };
}

}
