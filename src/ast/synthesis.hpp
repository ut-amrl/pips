// Copyright (c) Jarrett Holtz. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <z3++.h>
#include "Python.h"

#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "ast.hpp"

using namespace std;

namespace AST {

    ast_ptr PredicateL2(const vector<Example>& examples,
        const vector<ast_ptr>& ops,
        ast_ptr sketch,
        const pair<string, string>& transition,
        const double min_accuracy, float* best_score);

    ast_ptr ldipsL2(ast_ptr candidate,
        const vector<Example>& examples,
        const vector<ast_ptr>& ops,
        const pair<string, string>& transition,
        const float min_accuracy,
        ast_ptr best_program,
        float* best_score);

    vector<ast_ptr> ldipsL3(const vector<Example>& demos,
        const vector<pair<string, string>>& transitions,
        const vector<ast_ptr> lib,
        const int sketch_depth,
        const float min_accuracy,
        const string& output_path);

    void emdipsL3(const vector<Example> &demos,
        const vector<pair<string, string>> &transitions,
        vector<ast_ptr>& solution_preds,
        vector<float>& solution_loss,
        vector<ast_ptr>& sketches,
        vector<ast_ptr>& current_solutions,
        vector<ast_ptr>& gt_truth,
        const vector<float> max_error,
        const string &output_path,
        const uint32_t batch_size,
        const uint32_t max_enum,
        const bool use_gt,
        PyObject* pFunc);

    void emdipsL3(const vector<Example> &demos,
        const vector<pair<string, string>> &transitions,
        vector<ast_ptr>& solution_preds,
        vector<float>& solution_loss,
        vector<ast_ptr>& sketches,
        const vector<float>& max_error,
        const string &output_path,
        const uint32_t batch_size,
        const uint32_t max_enum,
        PyObject* pFunc);

    void SRTR(const vector<Example>& demos,
        const vector<ast_ptr>& programs,
        const vector<pair<string, string>>& transitions,
        const string& output_path);

    void DIPR(const vector<Example>& demos,
        const vector<ast_ptr>& programs,
        const vector<pair<string, string>>& transitions,
        const vector<ast_ptr> lib,
        const int sketch_depth,
        const float min_accuracy,
        const string& output_path);

}  // namespace AST
