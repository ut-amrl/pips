#include <dlfcn.h>
#include <gflags/gflags.h>
#include <z3++.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "ast.hpp"
#include "enumeration.hpp"
#include "library_functions.hpp"
#include "parsing.hpp"
#include "visitors/interp_visitor.hpp"
#include "visitors/print_visitor.hpp"
#include "sketches/sketches.hpp"

DEFINE_string(ex_file, "", "Examples file");
DEFINE_string(lib_file, "", "Operation library file");
DEFINE_uint32(feat_depth, 3, "Maximum enumeration depth for features.");
DEFINE_uint32(sketch_depth, 3, "Maximum enumeration depth for sketch.");
DEFINE_double(min_accuracy, 1.0,
              "What proportion of examples should be SAT to declare victory?");
DEFINE_bool(write_features, false, "Write all enumerated features to a file");
DEFINE_string(feature_file, "features.txt", "File to write features to");
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

  if (FLAGS_ex_file == "") {
    cout << "No examples file provided!" << endl;
    return EXIT_FAILURE;
  }

  if (FLAGS_lib_file == "") {
    cout << "No library file provided!" << endl;
    return EXIT_FAILURE;
  }

  // Read in the Examples
  // Also obtains variables that can be used (roots for synthesis)
  // and the possible input->output state pairs.
  unordered_set<Var> variables;
  unordered_set<std::pair<string,string>, pair_hash> transitions;
  vector<Example> examples = ReadExamples(FLAGS_ex_file,
      variables,
      &transitions);

  // Adding Library Function Definitions
  vector<FunctionEntry> library = ReadLibrary(FLAGS_lib_file);

  // Turning variables into roots
  vector<ast_ptr> inputs, roots;
  for (const Var& variable : variables) {
    roots.push_back(make_shared<Var>(variable));
  }

  // Enumerate features up to a fixed depth
  vector<Signature> signatures;
  vector<ast_ptr> ops = AST::RecEnumerate(roots, inputs, examples, library,
                                          FLAGS_feat_depth, &signatures);

  // Print out Roots, library, etc.

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

  cout << "----Library----" << endl;
  for (auto& func : library) {
    cout << func << endl;
  }
  cout << endl;

  cout << "---- Number Enumerated ----" << endl;
  cout << ops.size() << endl << endl;

  // Enumerate possible sketches
  const auto sketches = AST::EnumerateSketches(FLAGS_sketch_depth);
  // For each input/output pair
  for (const auto& transition : transitions) {
    for (const auto& sketch : sketches) {
      cout << sketch << endl;
      ast_ptr program =
        SolvePredicate(examples, ops, sketch, transition, FLAGS_min_accuracy);
    }
  }
}
