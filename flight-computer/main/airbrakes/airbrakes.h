#pragma once

#include "errors.h"
#include "gps/gps.h"
#include "i2c/BMI323.h"
#include "i2c/BMP581.h"
#include "i2c/high_g_accel.h"
#include "motor/motor.h"

#include "sd.h"

namespace seds {
    using namespace seds::errors;

    class Airbrakes {
    public:
        Airbrakes(Airbrakes&&) = default;
        Airbrakes& operator=(Airbrakes&&) = default;
        Airbrakes(Airbrakes const&) = delete;
        Airbrakes& operator=(Airbrakes) = delete;

        [[nodiscard]]
        static Expected<Airbrakes> create(std::shared_ptr<Barometer> b1, std::shared_ptr<Barometer> b2, BMI323 im, HighGAccel high_g, SDCard s, std::optional<GPS> gps);

        void run_steps();

    private:
        Airbrakes(std::shared_ptr<Barometer> b1, std::shared_ptr<Barometer> b2, BMI323 im, HighGAccel high_g, Motor m, SDCard s, std::optional<GPS> gps) : 
            baro1(b1), 
            baro2(b2), 
            gps(std::nullopt), 
            imu(std::make_shared<BMI323>(std::move(im))), 
            high_g_accel(std::make_shared<HighGAccel>(std::move(high_g))),
            motor(std::move(m)), 
            sd(std::move(s)) 
            {
                if (gps.has_value()) {
                    this->gps = std::make_shared<GPS>(std::move(gps).value());
                }
            };

        std::shared_ptr<Barometer> baro1;
        std::shared_ptr<Barometer> baro2;

        std::optional<std::shared_ptr<GPS>> gps;
        std::shared_ptr<BMI323> imu;
        std::shared_ptr<HighGAccel> high_g_accel;
        Motor motor;
        SDCard sd;
        
    };
}