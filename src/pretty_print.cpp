#include <dlfcn.h>
#include <gflags/gflags.h>
#include <z3++.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <algorithm>
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

// DEFINE_string(ex_file, "", "Examples file");
// DEFINE_string(lib_file, "ops/social_test.json", "Operation library file");
// DEFINE_string(output, "synthd/dipsl3/", "Operation library file");
// DEFINE_uint32(feat_depth, 3, "Maximum enumeration depth for features.");
// DEFINE_uint32(sketch_depth, 3, "Maximum enumeration depth for sketch.");
// DEFINE_uint32(window_size, 3, "Size of sliding window to subsample demonstrations with.");
// DEFINE_double(min_accuracy, 1.0,
              // "What proportion of examples should be SAT to declare victory?");
// DEFINE_bool(write_features, false, "Write all enumerated features to a file");
// DEFINE_string(feature_file, "features.txt", "File to write features to");
DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");
DEFINE_bool(debug, false, "Enable Debug Printing");

DEFINE_string(policy, "synthd/dips-window/", "Directory containing components of sketch.");

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

// bool ExistsFile(const string& filename) {
  // ifstream infile(filename);
  // return infile.good();
// }

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  // Read in the Examples
  // Also obtains variables that can be used (roots for synthesis)
  // and the possible input->output state pairs.
  unordered_set<Var> variables;
  vector<std::pair<string,string>> transitions_loaded;

  const vector<ast_ptr> branch_progs =
      LoadSketches(FLAGS_policy, &transitions_loaded);
  cout << endl << endl;
  for (size_t i = 0; i < transitions_loaded.size(); ++i) {
    const auto transition = transitions_loaded[i];
    ast_ptr pred = branch_progs[i];
    std::stringstream ss;
    ss << pred;
    const string pred_str = ss.str();
    string pretty = "if (state_ == " + transition.first + " && " + pred_str;
    pretty += "):\n";
    pretty += "  return " + transition.second;
    cout << pretty << endl;
  }
}
