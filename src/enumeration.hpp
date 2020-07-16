#pragma once

#include "ast.hpp"

#include <string>
#include <vector>

namespace AST {

struct FunctionSig {
  std::vector<Type> input_types_;
  std::vector<Dimension> input_dims_;
  Type output_type_;
  Dimension output_dim_;
};

struct FunctionEntry {
  std::string op_;
  std::vector<Type> input_types_;
  std::vector<Dimension> input_dims_;
  Type output_type_;
  Dimension output_dim_;
};

std::vector<ast_ptr> GetLegalOps(ast_ptr node, std::vector<ast_ptr> input,
                                 const std::vector<FunctionEntry>& library);

std::vector<ast_ptr> Enumerate(const std::vector<ast_ptr>& roots,
                               const std::vector<ast_ptr>& inputs,
                               const std::vector<FunctionEntry>& library);

std::vector<ast_ptr> RecEnumerate(const std::vector<ast_ptr>& roots,
                                  const std::vector<ast_ptr>& inputs,
                                  const std::vector<Example>& examples,
                                  const std::vector<FunctionEntry>& library,
                                  const int depth,
                                  std::vector<std::vector<float>>* signatures);

template <typename T>
bool IndexInVector(const std::vector<T>& vec, const T& element, int* index) {
  // Find given element in vector
  auto it = std::find(vec.begin(), vec.end(), element);
  if (it != vec.end()) {
    *index = distance(vec.begin(), it);
    return true;
  }
  *index = -1;
  return false;
}

} // namespace AST