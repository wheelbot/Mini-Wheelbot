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

#include <iostream>
#include <nlohmann/json.hpp>
#include <array>

#include "wheelbot/config.hpp"

using json = nlohmann::json;

int main() {
    Config::MainConfig main_config {
        {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f},
        { {12.0f, 13.0f, 14.0f, 15.0f}, {16.0f, 17.0f, 18.0f, 19.0f} }
    };

    json j = main_config;

    std::cout << j.dump(4) << std::endl;

    auto main_config2 = j.get<Config::MainConfig>();

    std::cout << "m_WR: " << main_config2.ampc_config.delta_m_WR << ", Kroll[0]: " << main_config2.balancing_config.K_roll[0] << std::endl;

    return 0;
}
