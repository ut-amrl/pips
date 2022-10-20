// Copyright (c) Jarrett Holtz. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <z3++.h>

#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <queue>

#include "utils/nd_bool_array.hpp"
#include "ast.hpp"

using namespace std;

namespace AST {

// Basically a breadth-first search of all combination of indices of the ops
// vector starting at {0, 0, ..., 0}
class index_iterator {
public:
    index_iterator(size_t op_count, size_t holes_to_fill)
        : op_count_(op_count),
        holes_to_fill_(holes_to_fill),
        visited_(nd_bool_array(
            vector<size_t>(holes_to_fill_, op_count_))) {  // :^)
        // Some checks to make sure what we're doing is sensible.
        if (op_count_ == 0) {
            throw std::invalid_argument("no ops!");
        }
        if (holes_to_fill_ == 0) {
            throw std::invalid_argument("no holes to fill!");
        }

        // make the initial list of however many 0s
        vector<size_t> initial_coords;
        for (size_t i = 0; i < holes_to_fill_; ++i) {
            initial_coords.push_back(0);
        }

        // Add it to the queue to start BFS
        indicies_.push(initial_coords);
    }

    vector<size_t> next() {
        // Get the current lowest priority index list
        vector<size_t> current = indicies_.front();
        indicies_.pop();

        // Add neighbors of this index list to the queue
        for (size_t i = 0; i < holes_to_fill_; ++i) {
            // if we've reached an edge, don't do anything
            if (current[i] == op_count_ - 1) {
                continue;
            }

            // Copy current index list, increase one index by 1, add it to the
            // queue, and mark it as visited so it's not added again.
            vector<size_t> copy = current;
            copy[i] += 1;
            if (!visited_.get(copy)) {
                indicies_.push(copy);
                visited_.set(copy, true);
            }
        }
        return current;
    }

    bool has_next() const { return !indicies_.empty(); }

    vector<size_t> zeros() const { return vector<size_t>(holes_to_fill_, 0); }

private:
    const size_t op_count_;
    const size_t holes_to_fill_;
    std::queue<vector<size_t>> indicies_;
    nd_bool_array visited_;
};

typedef unordered_map<string, ast_ptr> Model;
typedef vector<pair<ast_ptr, SymEntry>> Sketch;
typedef vector<SymEntry> Signature;

struct FunctionSig {
  vector<Type> input_types_;
  vector<Dimension> input_dims_;
  Type output_type_;
  Dimension output_dim_;
};

struct FunctionEntry {
  string op_;
  vector<Type> input_types_;
  vector<Dimension> input_dims_;
  Type output_type_;
  Dimension output_dim_;
};

ostream& operator<<(ostream& os, const FunctionEntry& fe);

vector<ast_ptr> GetLegalOps(ast_ptr node, vector<ast_ptr> input,
                                 const vector<FunctionEntry>& library);

vector<ast_ptr> EnumerateSketches(int depth);
vector<ast_ptr> EnumerateSketchesHelper(int depth);

ast_ptr ExtendPred(ast_ptr base, ast_ptr pos_sketch, ast_ptr neg_sketch,
    const float& pos, const float& neg);

vector<ast_ptr> Enumerate(const vector<ast_ptr>& roots,
                               const vector<ast_ptr>& inputs,
                               const vector<FunctionEntry>& library);

vector<ast_ptr> RecEnumerateLogistic(const vector<ast_ptr>& roots,
                                  const vector<ast_ptr>& inputs,
                                  const vector<Example>& examples,
                                  const vector<FunctionEntry>& library,
                                  const int depth,
                                  vector<Signature>* signatures);

vector<ast_ptr> RecEnumerate(const vector<ast_ptr>& roots,
                                  const vector<ast_ptr>& inputs,
                                  const vector<Example>& examples,
                                  const vector<FunctionEntry>& library,
                                  const int depth,
                                  vector<Signature>* signatures);

ast_ptr FillFeatureHoles(ast_ptr sketch, const vector<size_t> &indicies, const vector<ast_ptr> &ops);

vector<ast_ptr> EnumerateL3(vector<ast_ptr>& lib, int sketch_depth);

// vector<ast_ptr> SolveConditional(const vector<Example>& examples,
                                      // const vector<ast_ptr>& ops,
                                      // const Sketch& sketch,
                                      // double min_accuracy);

// ast_ptr SolvePredicate(const vector<Example>& examples,
    // const vector<ast_ptr>& ops,
    // const ast_ptr& sketch,
    // const pair<string, string>& transition,
    // double min_accuracy,
    // float* solved);

template <typename T>
bool IndexInVector(const vector<T>& vec, const T& element, int* index) {
  // Find given element in vector
  auto it = find(vec.begin(), vec.end(), element);
  if (it != vec.end()) {
    *index = distance(vec.begin(), it);
    return true;
  }
  *index = -1;
  return false;
}

double CheckModelAccuracy(const ast_ptr& cond,
                          const unordered_set<Example>& yes,
                          const unordered_set<Example>& no);

double CheckModelAccuracy(const ast_ptr& cond,
                          const unordered_set<Example>& yes,
                          const unordered_set<Example>& no,
                          float* pos,
                          float* neg);

vector<Example> FilterExamples(const vector<Example>& examples,
    pair<string, string> transition);

void SplitExamples(const vector<Example>& examples,
    pair<string, string> transition,
    unordered_set<Example>* yes, unordered_set<Example>* no);

void SplitExamplesVector(const vector<Example>& examples,
    pair<string, string> transition,
    vector<Example>* yes, vector<Example>* no);

float ScorePredicate(ast_ptr pred,
    const pair<string, string>& transition,
    const vector<Example>& examples, float* pos, float* neg);

vector<ast_ptr> RelativesOnly(const vector<ast_ptr>& ops);
}  // namespace AST
