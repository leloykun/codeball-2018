#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _NEURAL_NETWORK_H_
#define _NEURAL_NETWORK_H_

#include "Matrix.h"

struct NeuralNetwork {
  std::vector<Matrix> layers;
  Matrix *input;
  Matrix *output;
  std::vector<Matrix> weights;
  std::vector<Matrix> biases;

  NeuralNetwork() { }
  NeuralNetwork(int input_size, int num_hidden, int output_size) {
    
  }

  std::vector<double> calc_outputs(const std::vector<double> &inputs);
};

#endif
