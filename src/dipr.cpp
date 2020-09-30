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
DEFINE_string(lib_file, "ops/social_test.json", "Operation library file");
DEFINE_string(sketch_dir, "synthd/mpdm_1/", "Directory containing components of sketch.");
DEFINE_uint32(feat_depth, 3, "Maximum enumeration depth for features.");
DEFINE_uint32(sketch_depth, 3, "Maximum enumeration depth for sketch.");
DEFINE_uint32(window_size, 3, "Size of sliding window to subsample demonstrations with.");
DEFINE_double(min_accuracy, 1.0,
              "What proportion of examples should be SAT to declare victory?");
DEFINE_bool(write_features, false, "Write all enumerated features to a file");
DEFINE_string(feature_file, "features.txt", "File to write features to");
DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");
DEFINE_bool(debug, false, "Enable Debug Printing");

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

  examples = WindowExamples(examples, FLAGS_window_size);

  // Turning variables into roots
  vector<ast_ptr> inputs, roots;
  for (const Var& variable : variables) {
    roots.push_back(make_shared<Var>(variable));
  }

  cout << "----Roots----" << endl;
  for (auto& node : roots) {
    cout << node << endl;
  }
  cout << endl;

  cout << "----Transitions----" << endl;
  for (auto& trans : transitions) {
    cout << trans.first << "->" << trans.second << endl;
  }
  cout << endl;

  // Loading Library Function Definitions
  vector<FunctionEntry> library = ReadLibrary(FLAGS_lib_file);

  cout << "----Library----" << endl;
  for (auto& func : library) {
    cout << func << endl;
  }
  cout << endl;

  // Enumerate features up to a fixed depth
  vector<Signature> signatures;
  vector<ast_ptr> ops = AST::RecEnumerate(roots, inputs, examples, library,
                                          FLAGS_feat_depth, &signatures);

  if (FLAGS_debug) {
      cout << "---- Features Synthesized ----" << endl;
      for (auto& feat : ops) {
          cout << feat << endl;
      }
      cout << endl;
  }

  cout << "---- Number of Features Enumerated ----" << endl;
  cout << ops.size() << endl << endl;
  cout << endl;

  // Load the Existing Sketches
  vector<std::pair<string, string>> branches;
  const vector<ast_ptr> branch_sketches = LoadSketches(FLAGS_sketch_dir, &branches);

  // Enumerate possible sketch extensions
  const auto sketches = AST::EnumerateSketches(FLAGS_sketch_depth);

  // For each input/output pair
  for (size_t i = 0; i < branch_sketches.size(); ++i) {
    const auto transition = branches[i];
    ast_ptr initial = sketches[i];
    cout << transition.first << "->" << transition.second << endl;
    cout << "Starting Sketch" << initial << endl;

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
    float pos = 0;
    float neg = 0;
    float current_best = CheckModelAccuracy(initial, yes, no, &pos, &neg);
    cout << "Initial Score" << current_best << endl;
    if (current_best >= FLAGS_min_accuracy) {
      cout << "Sufficient Performance, Skipping" << endl;
      break;
    }

    // Search all possible extensions of the current program with enumerated
    // conditional sketches.
    ast_ptr current_solution = nullptr;
    for (const auto& sketch : sketches) {
      // Extend the Sketch
      // TODO(jaholtz) iterative over both sketches separately,
      // many candidate programs, but technically more correct.
      ast_ptr candidate = ExtendPred(initial, sketch, sketch, pos, neg);
      // Find best performing completion of current sketch
      float solved = 0.0;
      ast_ptr solution =
        SolvePredicate(examples,
            ops, candidate, transition, FLAGS_min_accuracy, &solved);

      // If the perfomance meets needs, stop searching
      // otherwise, keep if beats best performing solution.
      if (solved >= FLAGS_min_accuracy) {
        current_best = solved;
        current_solution = solution;
        break;
      } else if (solved > current_best) {
        current_best = solved;
        current_solution = solution;
      }

      Z3_reset_memory();
      cout << "Score: " << current_best << endl;
    }
    // Write the solution out to a file.
    cout << current_solution << endl;
    ofstream output_file;
    const string output_name =
      "synthd/repaired/" + transition.first + "_" + transition.second + ".json";
    output_file.open(output_name);
    const json output = current_solution->ToJson();
    output_file << std::setw(4) << output << std::endl;
    output_file.close();

    cout << endl;
  }
}
