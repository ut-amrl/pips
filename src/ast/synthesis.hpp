// Copyright (c) Jarrett Holtz. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <z3++.h>

#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "ast.hpp"

namespace AST {

  ast_ptr  PredicateL2(const std::vector<Example>& examples,
      const std::vector<ast_ptr>& ops,
      ast_ptr sketch,
      const std::pair<std::string, std::string>& transition,
      const double min_accuracy, float* best_score);

  ast_ptr ldipsL2(ast_ptr candidate,
    const std::vector<Example>& examples,
    const std::vector<ast_ptr>& ops,
    const std::pair<std::string, std::string>& transition,
    const float min_accuracy,
    ast_ptr best_program,
    float* best_score);

  void ldipsL3(const std::vector<Example>& demos,
      const std::vector<std::pair<std::string, std::string>>& transitions,
      const std::vector<ast_ptr> lib,
      const int sketch_depth,
      const float min_accuracy,
      const std::string& output_path);

void RulePIPS(const std::vector<std::vector<std::vector<Example>>>& positive,
              const std::vector<std::vector<std::vector<Example>>>& negative,
              const std::vector<ast_ptr> sketches, const std::vector<ast_ptr> lib,
              const float min_accuracy, const std::string& output_path);

  void SRTR(const std::vector<Example>& demos,
      const std::vector<ast_ptr>& programs,
      const std::vector<std::pair<std::string, std::string>>& transitions,
      const std::string& output_path);

  void DIPR(const std::vector<Example>& demos,
      const std::vector<ast_ptr>& programs,
      const std::vector<std::pair<std::string, std::string>>& transitions,
      const std::vector<ast_ptr> lib,
      const int sketch_depth,
      const float min_accuracy,
      const std::string& output_path);

std::vector<ast_ptr> EnumerateCompletions(const ast_ptr sketch, const std::vector<ast_ptr>& expressions);

}  // namespace AST
