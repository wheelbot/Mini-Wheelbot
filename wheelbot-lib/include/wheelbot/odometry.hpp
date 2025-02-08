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

#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <tuple>
#include <cmath>

class Odometry {
public:

    explicit
    Odometry(const float wheel_radius) : x_(0.0), y_(0.0), yaw_(0.0), radius_(wheel_radius), prev_wheel_angle_(0.0) {}

    // Update function computes the world position
    std::tuple<float, float, float, float> update(
        const float body_yaw,
        const float wheel_angle,
        const float wheel_angular_velocity) {

        const float delta_wheel_angle = wheel_angle - prev_wheel_angle_;
        prev_wheel_angle_ = wheel_angle;

        const float displacement = delta_wheel_angle * radius_;
        velocity_ = wheel_angular_velocity * radius_;

        // Update yaw and integrate to world position
        yaw_ = body_yaw;
        x_ += std::cos(yaw_) * displacement;
        y_ += std::sin(yaw_) * displacement;

        return std::make_tuple(x_, y_, velocity_, yaw_);
    }

private:
    float x_;                // x position in the world frame
    float y_;                // y position in the world frame
    float yaw_;              // current yaw orientation of the body
    float velocity_;         // velocity in body direction
    float radius_;           // radius of the wheel
    float prev_wheel_angle_; // previous wheel angle
};

#endif // INPUTSERVER_HPP
