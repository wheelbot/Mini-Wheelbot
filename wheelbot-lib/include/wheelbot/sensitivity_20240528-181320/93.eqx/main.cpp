#include <iostream>
#include <fstream>
#include <array>
#include <string>

#include <chrono>

#include "neural_network.hpp"

#include <eigen3/Eigen/Dense>


int main() {
    Eigen::Vector<float, 10 > input{ {0.7089051946170651, 0.3912781539834793, 0.615536704555418, 0.538006098277997, 0.6881768517684153, 0.021634195508049126, 0.9461266254083219, 0.8651395966572244, 0.36313522676622334, 0.8881810661638261} };
    std::cout << "input       = "<< input.transpose() << std::endl;


    const auto start_inference = std::chrono::high_resolution_clock::now();

    const auto input_normalized = embedded_nn_inference::sensitivity::normalize_input(input);
    const auto output_normalized = embedded_nn_inference::sensitivity::call_nn(input_normalized);
    const auto output = embedded_nn_inference::sensitivity::denormalize_output(output_normalized);

    const auto output_reshaped = embedded_nn_inference::sensitivity::reshape_output(output);

    const auto end_inference = std::chrono::high_resolution_clock::now();


    std::cout << "output      = " << output.transpose() << std::endl << std::endl;
    Eigen::Vector<float, 22 > test_result{ {0.0029785368, -8.5336505e-06, 3.9334717, -12.937984, -0.043059383, 0.25568905, 1.6127037, 0.018246748, 0.010548394, 0.0708148, -5.561389e-08, 0.0006792024, -8.7406486e-05, 0.31462392, -1.3200111, -0.010098666, 0.0004843697, -0.12221983, 0.002130976, 0.00013616681, 0.015000194, 8.221122e-09} };
    std::cout << "test_result = " << test_result.transpose() << std::endl << std::endl;

    std::cout << "output_resh = " << output_reshaped << std::endl << std::endl;

    std::chrono::duration<double> total_inf_time{end_inference-start_inference};
    std::cout << "Inference took " << total_inf_time << std::endl << std::endl;

    }
