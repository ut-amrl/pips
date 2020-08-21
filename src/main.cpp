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

DEFINE_string(exs_file, "", "Examples file");
DEFINE_string(ops_file, "", "Operation library file");
DEFINE_uint32(max_depth, 3, "Maximum enumeration depth");
DEFINE_double(min_accuracy, 1.0,
              "What proportion of examples should be SAT to declare victory?");
DEFINE_bool(write_features, false, "Write all enumerated features to a file");
DEFINE_string(feature_file, "features.txt", "File to write features to");

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

  if (FLAGS_exs_file == "") {
    cout << "No examples file provided!" << endl;
    exit(1);
  }

  if (FLAGS_ops_file == "") {
    cout << "No ops file provided!" << endl;
    exit(1);
  }

  unordered_set<Var> variables;
  vector<Example> examples = ReadExamples(FLAGS_exs_file, variables);

  // Adding Library Function Definitions
  vector<FunctionEntry> library = ReadLibrary(FLAGS_ops_file);

  vector<ast_ptr> inputs, roots;
  for (const Var& variable : variables) {
    roots.push_back(make_shared<Var>(variable));
  }

  vector<Signature> signatures;
  vector<ast_ptr> ops = AST::RecEnumerate(roots, inputs, examples, library,
                                          FLAGS_max_depth, &signatures);
  cout << "----Roots----" << endl;
  for (auto& node : roots) {
    cout << node << endl;
  }
  cout << endl;

  cout << "----Library----" << endl;
  for (auto& func : library) {
    cout << func << endl;
  }
  cout << endl;

  if (FLAGS_write_features) {
    ofstream feature_file;
    feature_file.open(FLAGS_feature_file);
    for (auto& op : ops) {
      feature_file << op << '\n';
    }
    feature_file.close();
  }

  cout << "---- Number Enumerated ----" << endl;
  cout << ops.size() << endl << endl;

  Param p1("p1", {1, 0, 0}, NUM);
  Param p2("p2", {1, 0, 0}, NUM);
  Feature f1("f1", {1, 0, 0}, NUM);
  Feature f2("f2", {1, 0, 0}, NUM);
  Feature f3("f3", {0, 0, 0}, BOOL);

  /*
  if (feature1 >= param1) {
    go forward
  } else if (feature1 < param1 && feature2 > param2) {
    go left
  } else {
    go right
  }
  */
  BinOp cond1(make_shared<Feature>(f1), make_shared<Param>(p1), "Gte");
  BinOp cond2(make_shared<BinOp>(BinOp(make_shared<Feature>(f1),
                                       make_shared<Param>(p1), "Lt")),
              make_shared<BinOp>(BinOp(make_shared<Feature>(f2),
                                       make_shared<Param>(p2), "Gt")),
              "And");

  const Sketch sketch = {
      std::make_pair(make_shared<BinOp>(cond1), SymEntry(0.0f)),
      std::make_pair(make_shared<BinOp>(cond2), SymEntry(1.0f)),
      std::make_pair(make_shared<Feature>(f3), SymEntry(2.0f))};

  cout << "---- Sketch ----" << endl;
  for (const auto& p : sketch) {
    cout << p.first << " --> " << p.second << endl;
  }
  cout << endl;

  cout << "---- Solving ----" << endl;
  vector<ast_ptr> program =
      SolveConditional(examples, ops, sketch, FLAGS_min_accuracy);
  cout << endl;

  cout << "---- Program conditions ----" << endl;
  for (const ast_ptr& cond : program) {
    cout << cond << endl;
  }
  cout << endl;
}
