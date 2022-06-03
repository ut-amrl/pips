#include <dlfcn.h>
#include <gflags/gflags.h>
#include <z3++.h>

#include <algorithm>
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
#include "ast/synthesis.hpp"
#include "visitors/interp_visitor.hpp"
#include "visitors/print_visitor.hpp"

DEFINE_string(ex_file, "/home/hja1ply/workspace/pips/highway-examples/toy/",
              "Examples file");
DEFINE_string(lib_file, "ops/driving.json", "Operation library file");
DEFINE_string(out_dir, "synthd/rulePIPS/", "Output File");
DEFINE_uint32(feat_depth, 2, "Maximum enumeration depth for features.");
DEFINE_uint32(sketch_depth, 1, "Maximum enumeration depth for sketch.");
DEFINE_uint32(window_size, -1,
              "Size of sliding window to subsample demonstrations with.");
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
using Eigen::Vector2f;
using std::cout;
using std::endl;
using std::ifstream;
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

  // Read in the Examples and Variables for synthesis
  // TODO(jaholtz, shorterm) - Create one example per car in example.
  // TODO(jaholtz) - Create a more general handling for these mappings
  // (arbitrary maps of ops to lists)
  unordered_set<Var> variables;
  vector<vector<vector<Example>>> positive, negative;
  vector<Example> examples;
  ReadTrajExamples(FLAGS_ex_file, variables, positive, negative);

  for (auto example : positive) {
    for (auto trajectory : example) {
      for (auto observation : trajectory) {
        examples.push_back(observation);
      }
    }
  }

  for (auto example : negative) {
    for (auto trajectory : example) {
      for (auto observation : trajectory) {
        if (examples.size() < 1000) {
          examples.push_back(observation);
        }
      }
    }
  }

  // Turning variables into roots
  // TODO(jaholtz) - Make sure the roots are correct
  vector<ast_ptr> inputs, roots;
  cout << "Number of Variables: " << variables.size() << endl;
  cout << "Number of Positive: " << positive.size() << endl;
  cout << "Number of Negative: " << negative.size() << endl;
  cout << "Number of Timesteps: " << examples.size() << endl;
  for (const Var& variable : variables) {
    if (variable.name_ != "mistake" && variable.name_ != "action" &&
        variable.name_ != "presence") {
      roots.push_back(make_shared<Var>(variable));
    }
  }

  cout << "----Roots----" << endl;
  for (auto& node : roots) {
    cout << node << endl;
  }
  cout << endl;

  // Loading Library Function Definitions
  // TODO(jaholtz) Create the libary file for this case
  vector<FunctionEntry> library = ReadLibrary(FLAGS_lib_file);

  cout << "----Library----" << endl;
  for (auto& func : library) {
    cout << func << endl;
  }
  cout << endl;

  // Enumerate features up to a fixed depth
  // TODO(jaholtz) Make sure signature pruning is correctly handled here...
  vector<Signature> signatures;
  vector<ast_ptr> ops = AST::RecEnumerate(roots, inputs, examples, library,
                                          FLAGS_feat_depth, &signatures);
  
  Feature fX("fX", {0, 0, 0}, NUM);
  Feature fY("fY", {0, 0, 0}, NUM);
  BinOp less_op(make_shared<Feature>(fX), make_shared<Feature>(fY), "Lt", BOOL, {0, 0, 0});
  vector<ast_ptr> compare = AST::EnumerateCompletions(std::make_shared<BinOp>(less_op), ops); 
  vector<ast_ptr> legal_compare;
  for (const ast_ptr comparison : compare) {
    if (comparison->IsLegal()) {
      legal_compare.push_back(comparison); 
    }
  }

  // Enumerate Sketches up to a fixed depth
  vector<ast_ptr> sketches = AST::EnumerateRuleSketches(FLAGS_sketch_depth);

  if (FLAGS_debug) {
    cout << "---- Features Synthesized ----" << endl;
    for (auto& feat : legal_compare) {
      cout << feat << endl;
    }
    cout << endl;

    cout << "---- Sketches To Consider ----" << endl;
    for (auto& sketch : sketches) {
      cout << sketch << endl;
    }
    cout << endl;
  }

  cout << "---- Number of Features Enumerated ----" << endl;
  cout << legal_compare.size() << endl << endl;
  cout << "---- Number of Positive Trajectories ----" << endl;
  cout << positive.size() << endl << endl;
  cout << "---- Number of Negative Trajectories ----" << endl;
  cout << negative.size() << endl << endl;
  cout << endl;

  // TODO(jaholtz) write the primary loop and conversion into SMT constraints
  RulePIPS(positive, negative, sketches, legal_compare, 50, FLAGS_out_dir);
}