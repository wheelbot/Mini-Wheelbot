#include <iostream>
#include <fstream>
#include <array>
#include <string>

#include <chrono>

#include "neural_network.hpp"

#include <eigen3/Eigen/Dense>


int main() {
    Eigen::Vector<float, 10 > input{ {0.08610751956258134, 0.5130263355894465, 0.5014848469067751, 0.22283712831314206, 0.016452839619571225, 0.03726418546852095, 0.4486703170219426, 0.016440311545234843, 0.25129284063017077, 0.682703567280409} };
    std::cout << "input       = "<< input.transpose() << std::endl;


    const auto start_inference = std::chrono::high_resolution_clock::now();

    const auto input_normalized = embedded_nn_inference::action::normalize_input(input);
    const auto output_normalized = embedded_nn_inference::action::call_nn(input_normalized);
    const auto output = embedded_nn_inference::action::denormalize_output(output_normalized);

    const auto output_reshaped = embedded_nn_inference::action::reshape_output(output);

    const auto end_inference = std::chrono::high_resolution_clock::now();


    std::cout << "output      = " << output.transpose() << std::endl << std::endl;
    Eigen::Vector<float, 2 > test_result{ {0.068196766, 0.10010466} };
    std::cout << "test_result = " << test_result.transpose() << std::endl << std::endl;

    std::cout << "output_resh = " << output_reshaped << std::endl << std::endl;

    std::chrono::duration<double> total_inf_time{end_inference-start_inference};
    std::cout << "Inference took " << total_inf_time << std::endl << std::endl;

    }
