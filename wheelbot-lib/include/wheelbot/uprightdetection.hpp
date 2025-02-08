// MIT License

// Copyright (c) 2024 Henrik Hose
//               2024 Jan Weisgerber

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

#ifndef UPRIGHTDETECTION_HPP
#define UPRIGHTDETECTION_HPP

#include <eigen3/Eigen/Dense>

#include <cmath>


class UprightDetector{

    bool upright_ = false;
    bool was_upright_ = false;
    bool flip_upright_ = false;
    bool flip_was_upright_ = false;


    constexpr
    float deg2rad(const float deg) {
        return deg * M_PI / 180.0;
    }

public:
    UprightDetector(const Config::StandUpConfig standup_config_)
        : standup_config(standup_config_)
    {

    }

    Config::StandUpConfig standup_config;

    bool
    detect_latched(const Eigen::Vector<float, 10>& x){
        if ( (!is_upright() && !was_upright_) && ( (std::abs(x(1)) < deg2rad(2)) && (std::abs(x(2)) < deg2rad(2)) && (x.segment<2>(4).norm() < deg2rad(1)) )){
            upright_ = true;
            was_upright_ = true;
        }
        if ( (is_upright()) && ( (std::abs(x(1)) > deg2rad(18)) || (std::abs(x(2)) > deg2rad(45)) ) ) upright_ = false;
        return upright_;
    }

    // This function checks whether the robot is in the operating range
    // after a regular roll or pitch stand-up maneuver.
    bool
    detect_operating_range(const Eigen::Vector<float, 10>& x){
        if (((std::abs(x(1)) < deg2rad(standup_config.detection_angle_roll)) && (std::abs(x(2)) < deg2rad(standup_config.detection_angle_pitch)))){
            upright_ = true;
            was_upright_ = true;
        }
        if ( (is_upright()) && ( (std::abs(x(1)) > deg2rad(standup_config.detection_angle_roll + 5)) || (std::abs(x(2)) > deg2rad(standup_config.detection_angle_pitch + 5)) ) ) upright_ = false; //18
        return upright_;
    }

    // This function checks whether the robot is in the operating range
    // after a flip stand-up maneuver.
    bool
    detect_flip_operating_range(const Eigen::Vector<float, 10>& x){
        if (((std::abs(x(1)) < deg2rad(standup_config.flip_detection_angle_roll)) && (std::abs(x(2)) < deg2rad(standup_config.flip_detection_angle_pitch)))){
            flip_upright_ = true;
            flip_was_upright_ = true;
        }
        if ( (is_upright()) && ( (std::abs(x(1)) > deg2rad(standup_config.flip_detection_angle_roll + 5)) || (std::abs(x(2)) > deg2rad(standup_config.flip_detection_angle_pitch + 5)) ) ) flip_upright_ = false; //18
        return flip_upright_;
    }


    inline
    bool
    is_upright(){
        return upright_;
    }

    inline
    bool
    has_crashed(){
        return ( !upright_ ) && was_upright_;
    }

};

#endif // UPRIGHTDETECTION_HPP
