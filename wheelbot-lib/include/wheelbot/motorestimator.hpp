// MIT License

// Copyright (c) 2024 Henrik Hose

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef MOTORESTIMATOR_HPP
#define MOTORESTIMATOR_HPP

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <numbers>

class MotorEstimator{

private:

    static constexpr size_t nx = 3;
    static constexpr size_t nu = 1;
    static constexpr size_t ny = 1;

    float dT = 1e-3;

    const Eigen::Matrix<float, nx, nx> A{{1, dT, dT*dT/2},
                                          {0, 1, dT},
                                          {0, 0, 1} };

    Eigen::Vector<float, nx> K{9.91297949e-01,1.14737440e+02,6.59623054e+03};

    Eigen::Vector<float, nx> x{0,0,0};

    std::optional<float> y_last;
    double revs = 0;

public:
    void predict(){
        // Eigen::Matrix<float, nx, nx> APA = A*P*A.transpose();
        // P =  APA + Q;
        x = A*x;
    }

    void update(const float y){
        if ( !y_last.has_value() ) y_last = y;
        float y_delta = y - y_last.value();
        // abort if got weird value?
        if (abs(y_delta) > 2.f*std::numbers::pi_v<float>){
            return;
        }
        y_last = y;
        while(y_delta < -std::numbers::pi_v<float>){
            y_delta += 2.f*std::numbers::pi_v<float>;
        }
        while(y_delta > std::numbers::pi_v<float> ){
            y_delta -= 2.f*std::numbers::pi_v<float>;
        }
        revs += y_delta;
        x = x + K*(revs - x(0));
    }

    float get_propagated_est_angle(const float delay){
        #ifdef DEV_BUILD
        return 0;
        #endif
        return x(0) + delay * x(1);
    }

    float get_propagated_meas_angle(const float delay){
        #ifdef DEV_BUILD
        return 0;
        #endif
        return revs + delay * x(1); // + delay * delay / 2 * x(2);
    }

    float get_meas_angle(){
        #ifdef DEV_BUILD
        return 0;
        #endif
        return revs;
    }

    float get_velocity(){
        #ifdef DEV_BUILD
        return 0;
        #endif
        return x(1);
    }

    float get_propagated_velocity(const float delay){
        #ifdef DEV_BUILD
        return 0;
        #endif
        return x(1)+ delay * x(2);
    }

    float get_acceleration(){
        #ifdef DEV_BUILD
        return 0;
        #endif
        return x(2);
    }

};

#endif // MOTORESTIMATOR_HPP
