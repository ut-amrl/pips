#include <bits/stdint-uintn.h>
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
#include "visitors/fillhole_visitor.hpp"
#include "visitors/interp_visitor.hpp"
#include "visitors/print_visitor.hpp"
// #include "sketches/sketches.hpp"

DEFINE_string(ex_file, "merged.json", "Examples file");
DEFINE_string(lib_file, "ops/social_ref.json", "Operation library file");
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
using std::ifstream;
using std::string;
using std::unordered_map;
using std::unordered_set;
using std::vector;
using json = nlohmann::json;
using z3::context;
using z3::solver;

bool ExistsFile(const string& filename) {
  ifstream infile(filename);
  return infile.good();
}

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

  // examples = WindowExamples(examples, FLAGS_window_size);

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

  // Enumerate possible sketches
  const auto sketches = AST::EnumerateSketchesHelper(1);

  // For each input/output pair
  for (const auto& transition : transitions) {
    // Skipping already synthesized conditions, allows for very basic
    // checkpointing.
    const string output_name =
      "synthd/idips/" + transition.first + "_" + transition.second + ".json";
    if (ExistsFile(output_name)) {
      continue;
    }

    cout << transition.first << "->" << transition.second << endl;
    float current_best = 0.0;
    ast_ptr current_solution = nullptr;
    uint32_t iter = 0;
    bool improved = true;
    while (iter < FLAGS_sketch_depth && current_best < FLAGS_min_accuracy) {
      improved = false;
      for (const ast_ptr& sketch : sketches) {
        ast_ptr solution = current_solution;
        float score = 0.0;
        if (iter == 0) {
          solution =
            SolvePredicate(&examples,
                ops, sketch, transition, FLAGS_min_accuracy, &score);
        } else {
          AST::ResetParams(current_solution);
          BinOp ander(current_solution, sketch, "And");
          BinOp order(current_solution, sketch, "Or");
          ast_ptr and_ptr = make_shared<BinOp>(ander);
          ast_ptr or_ptr = make_shared<BinOp>(order);
          cout << "Current Sketches" << endl;
          cout << and_ptr << endl;
          cout << or_ptr << endl;
          float and_score = 0;
          float or_score = 0;
          ast_ptr and_solution =
            SolvePredicate(&examples,
                ops, and_ptr,
                transition, FLAGS_min_accuracy, &and_score);
          solution =
            SolvePredicate(&examples,
                ops, or_ptr,
                transition, FLAGS_min_accuracy, &score);
          // Just the one check, because we assign it to the or block by
          // default.
          if (and_score > or_score) {
            solution = and_solution;
            score = and_score;
          }
        }
        // If the perfomance meets needs, stop searching
        // otherwise, keep if beats best performing solution.
        if (score >= FLAGS_min_accuracy) {
          current_best = score;
          current_solution = solution;
          improved = true;
          break;
        } else if (score > current_best) {
          current_best = score;
          current_solution = solution;
          improved = true;
        }
        Z3_reset_memory();
        cout << "Score: " << current_best << endl;
        cout << "Current Solution: " << current_solution << endl;
      }
      iter++;
    }
    // Write the solution out to a file.
    cout << "Final Solution: " << current_solution << endl;
    ofstream output_file;
    output_file.open(output_name);
    const json output = current_solution->ToJson();
    output_file << std::setw(4) << output << std::endl;
    output_file.close();

    cout << endl;
  }
}
