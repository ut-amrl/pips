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
#include "ast/synthesis.hpp"
// #include "sketches/sketches.hpp"

DEFINE_string(ex_file, "merged.json", "Examples file");
DEFINE_string(lib_file, "ops/social_ref.json", "Operation library file");
DEFINE_string(sketch_dir, "cedrF/policies/ldips4/", "Directory containing components of sketch.");
DEFINE_uint32(feat_depth, 2, "Maximum enumeration depth for features.");
DEFINE_uint32(sketch_depth, 2, "Maximum enumeration depth for sketch.");
DEFINE_uint32(window_size, 5, "Size of sliding window to subsample demonstrations with.");
DEFINE_double(min_accuracy, 1.0,
              "What proportion of examples should be SAT to declare victory?");
DEFINE_bool(write_features, false, "Write all enumerated features to a file");
DEFINE_string(feature_file, "features.txt", "File to write features to");
DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");
DEFINE_bool(debug, true, "Enable Debug Printing");

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
using AST::SymEntry;
using AST::Type;
using AST::Var;
using AST::VEC;
using AST::PredicateL2;
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
  vector<std::pair<string,string>> transitions;
  vector<std::pair<string,string>> transitions_loaded;
  vector<Example> examples = ReadExamples(FLAGS_ex_file,
      variables,
      &transitions);

  std::reverse(transitions.begin(), transitions.end());

  cout << "Examples Loaded: " << examples.size() << endl << endl;

  // examples = WindowExamples(examples, FLAGS_window_size);

  // Turning variables into roots
  vector<ast_ptr> inputs, roots;
  for (const Var& variable : variables) {
    if (variable.name_ != "goal" && variable.name_ != "free_path" &&
        variable.name_ != "DoorState" && variable.name_ != "DoorPose") {
      roots.push_back(make_shared<Var>(variable));
    }
  }

  cout << "----Roots----" << endl;
  for (auto& node : roots) {
    cout << node << endl;
  }
  cout << endl;

  const vector<ast_ptr> branch_progs =
      LoadSketches(FLAGS_sketch_dir, &transitions_loaded);

  vector<ast_ptr> progs_sorted;

  // Sorting the transitions and programs based on the number
  // of examples
  cout << "Prog Size: " << branch_progs.size() << endl;
  for (size_t i = 0; i < transitions.size(); ++i) {
    const auto trans = transitions[i];
    const auto it =
        std::find(transitions_loaded.begin(), transitions_loaded.end(), trans);
    if (it != transitions_loaded.end()) {
      const int index = it - transitions_loaded.begin();
      progs_sorted.push_back(branch_progs[index]);
    } else {
      AST::Bool none(false);
      progs_sorted.push_back(make_shared<AST::Bool>(none));
    }
  }

  for (size_t i = 0; i < transitions_loaded.size(); ++i) {
    const auto trans = transitions_loaded[i];
    const auto prog = branch_progs[i];
    if (std::find(transitions.begin(), transitions.end(), trans)
        == transitions.end()) {
      transitions.insert(transitions.begin(), trans);
      progs_sorted.insert(progs_sorted.begin(), prog);
    }
  }

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

  // ops = RelativesOnly(ops);
  // Load the Existing Sketches
  // vector<std::pair<string, string>> branches;
  DIPR(examples,
      progs_sorted,
      transitions,
      ops,
      FLAGS_sketch_depth,
      FLAGS_min_accuracy,
      "synthd/dipr/");
}
