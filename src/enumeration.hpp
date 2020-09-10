#pragma once

#include <z3++.h>

#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "ast.hpp"

namespace AST {

typedef std::unordered_map<std::string, ast_ptr> Model;
typedef std::vector<std::pair<ast_ptr, SymEntry>> Sketch;
typedef std::vector<SymEntry> Signature;

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

std::ostream& operator<<(std::ostream& os, const FunctionEntry& fe);

std::vector<ast_ptr> GetLegalOps(ast_ptr node, std::vector<ast_ptr> input,
                                 const std::vector<FunctionEntry>& library);

std::vector<ast_ptr> EnumerateSketches(int depth);

std::vector<ast_ptr> Enumerate(const std::vector<ast_ptr>& roots,
                               const std::vector<ast_ptr>& inputs,
                               const std::vector<FunctionEntry>& library);

std::vector<ast_ptr> RecEnumerate(const std::vector<ast_ptr>& roots,
                                  const std::vector<ast_ptr>& inputs,
                                  const std::vector<Example>& examples,
                                  const std::vector<FunctionEntry>& library,
                                  const int depth,
                                  std::vector<Signature>* signatures);

std::vector<ast_ptr> SolveConditional(const std::vector<Example>& examples,
                                      const std::vector<ast_ptr>& ops,
                                      const Sketch& sketch,
                                      double min_accuracy);

ast_ptr SolvePredicate(const std::vector<Example>& examples,
    const std::vector<ast_ptr>& ops,
    const ast_ptr& sketch,
    const std::pair<std::string, std::string>& transition,
    double min_accuracy);

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

}  // namespace AST
