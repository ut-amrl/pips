#include <dlfcn.h>
#include <gflags/gflags.h>
#include <z3++.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "ast/ast.hpp"
#include "ast/enumeration.hpp"
#include "ast/library_functions.hpp"
#include "ast/parsing.hpp"
#include "visitors/interp_visitor.hpp"
#include "visitors/print_visitor.hpp"
// #include "sketches/sketches.hpp"

DEFINE_string(ex_file, "examples/merged.json", "Examples file");
DEFINE_uint32(window_size, 3, "Size of sliding window to subsample demonstrations with.");
DEFINE_double(min_accuracy, 1.0,
              "What proportion of examples should be SAT to declare victory?");
DEFINE_string(sketch_dir, "synthd/mpdm_1/", "Directory containing components of sketch.");
DEFINE_bool(debug, false, "Enable Debug Printing");
DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");

using AST::ast_ptr;
using AST::BinOp;
using AST::BOOL;
using AST::Dimension;
using AST::Example;
using AST::Feature;
using AST::FunctionEntry;
using AST::Model;
using AST::Num;
using AST::NUM;
using AST::Param;
using AST::Signature;
using AST::Sketch;
using AST::SolveConditional;
using AST::SymEntry;
using AST::Type;
using AST::Var;
using AST::VEC;
using Eigen::Vector2f;
using std::cout;
using std::endl;
using std::invalid_argument;
using std::make_shared;
using std::map;
using std::ofstream;
using std::string;
using std::unordered_map;
using std::unordered_set;
using std::vector;
using json = nlohmann::json;
using z3::context;
using z3::solver;

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  if (FLAGS_min_accuracy < 0.0)
    FLAGS_min_accuracy = 0.0;
  else if (FLAGS_min_accuracy > 1.0)
    FLAGS_min_accuracy = 1.0;

  // Read in the Examples
  // Also obtains variables that can be used (roots for synthesis)
  // and the possible input->output state pairs.
  unordered_set<Var> variables;
  unordered_set<std::pair<string,string>, pair_hash> transitions;
  vector<Example> examples = ReadExamples(FLAGS_ex_file,
      variables,
      &transitions);

  cout << "Examples Loaded" << endl;

  // TODO(jaholtz) Does SRTRing need this?
  // examples = WindowExamples(examples, FLAGS_window_size);

  // Turning variables into roots
  vector<ast_ptr> inputs, roots;
  for (const Var& variable : variables) {
    roots.push_back(make_shared<Var>(variable));
  }

  cout << "----Transitions----" << endl;
  for (auto& trans : transitions) {
    cout << trans.first << "->" << trans.second << endl;
  }
  cout << endl;

  // Load the Sketches
  vector<std::pair<string, string>> branches;
  const vector<ast_ptr> sketches = LoadSketches(FLAGS_sketch_dir, &branches);

  // For each branch in the existing sketch
  for (size_t i = 0; i < sketches.size(); ++i) {
    ast_ptr sketch = sketches[i];
    const auto transition = branches[i];
    cout << transition.first << "->" << transition.second << endl;
    cout << "Initial Sketch: " << sketch << endl;

    // Checking Initial accuracy
    const SymEntry out(transition.second);
    const SymEntry in(transition.first);
    // Split up all the examples into a "yes" set or a "no" set based on
    // whether the result for the example matches the current example's
    // behavior.
    unordered_set<Example> yes;
    unordered_set<Example> no;
    for (const Example& example : examples) {
      if (example.result_ == out && example.start_ == in) {
        yes.insert(example);
      } else if (example.start_ == in) {
        no.insert(example);
      }
    }
    float current_best = CheckModelAccuracy(sketch, yes, no);
    cout << "Initial Score: " << current_best << endl;

    // Find best performing completion of current sketch
    float solved = 0.0;
    ast_ptr solution =
      SolvePredicate(examples,
          {}, sketch, transition, FLAGS_min_accuracy, &solved);

    cout << "Adjusted Program: " << solution << endl;
    cout << "Final Score: " << solved << endl << endl;
    Z3_reset_memory();
  }
}
