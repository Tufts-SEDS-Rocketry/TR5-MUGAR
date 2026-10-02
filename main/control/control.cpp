#include "control.h"
#include <cmath>
#include <cstdint>


namespace seds {

constexpr double area = 0.01936;
constexpr double mass = 21.0;
constexpr double c = 0.115;
constexpr double c_init = 0.448;
constexpr double rho_init = 1.225;
constexpr double rho_coeff = -0.0001;
constexpr double g = 9.81;

constexpr uint32_t iters = 10;
constexpr double goal = 10000 * 0.3048;

double apogee(double altitude, double v_vertical, double u) {
    double cd = 0.5 * area * (rho_init + rho_coeff * altitude) * (c * u + c_init);
    return altitude + mass / (2.0 * cd) * std::log((v_vertical * v_vertical * cd + mass * g) / (mass * g));
}

double control(double altitude, double v_vertical) {
    if (apogee(altitude, v_vertical, 0.0) < goal) {
        return 0.0;
    }

    if (apogee(altitude, v_vertical, 1.0) > goal) {
        return 1.0;
    }

    double l = 0;
    double u = 1.0;

    for (int i = 0; i < iters; i++) {
        double mid = (l+u)/2.0;
        if (apogee(altitude, v_vertical, mid) > goal) {
            l = mid;
        } else {
            u = mid;
        }
    }

    return l;
}

}
