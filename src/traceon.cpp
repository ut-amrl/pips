#include <dlfcn.h>
#include <gflags/gflags.h>
#include <queue>
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
#include "visitors/trace_visitor.hpp"
#include "ast/synthesis.hpp"
#include "visitors/deepcopy_visitor.hpp"
#include "boost/dynamic_bitset.hpp"

DEFINE_string(ex_file, "big_ref.json", "Examples file");
DEFINE_string(ast_path, "synthd/ref_clone/", "Operation library file");
DEFINE_uint32(window_size, 3, "Size of sliding window to subsample demonstrations with.");

using AST::ast_ptr;
using AST::BinOp;
using AST::If;
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
using AST::String;
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

vector<std::pair<string,string>> trans_list_;
// Transition Function ASTs
ast_ptr ga_to_ga_;
ast_ptr ga_to_follow_;
ast_ptr ga_to_halt_;
ast_ptr ga_to_pass_;
ast_ptr follow_to_ga_;
ast_ptr follow_to_follow_;
ast_ptr follow_to_halt_;
ast_ptr follow_to_pass_;
ast_ptr halt_to_ga_;
ast_ptr halt_to_follow_;
ast_ptr halt_to_halt_;
ast_ptr halt_to_pass_;
ast_ptr pass_to_pass_;
ast_ptr pass_to_ga_;
ast_ptr pass_to_halt_;
ast_ptr pass_to_follow_;

const bool kDebug = true;

vector<std::pair<string,string>> LoadTransitionList(const string& path) {
  std::ifstream infile(path);
  vector<std::pair<string, string>> trans_list;
  string start, output;
  while(infile >> start >> output) {
    const std::pair<string, string> trans(start, output);
    trans_list.push_back(trans);
  }
  return trans_list;
}

void LoadSocialAST() {
  trans_list_ = LoadTransitionList(FLAGS_ast_path + "transitions.txt");
  ga_to_ga_ = LoadJson(FLAGS_ast_path + "GoAlone_GoAlone.json");
  ga_to_follow_ = LoadJson(FLAGS_ast_path + "GoAlone_Follow.json");
  ga_to_halt_ = LoadJson(FLAGS_ast_path + "GoAlone_Halt.json");
  ga_to_pass_ = LoadJson(FLAGS_ast_path + "GoAlone_Pass.json");
  follow_to_ga_ = LoadJson(FLAGS_ast_path + "Follow_GoAlone.json");
  follow_to_follow_ = LoadJson(FLAGS_ast_path + "Follow_Follow.json");
  follow_to_halt_ = LoadJson(FLAGS_ast_path + "Follow_Halt.json");
  follow_to_pass_ = LoadJson(FLAGS_ast_path + "Follow_Pass.json");
  halt_to_ga_ = LoadJson(FLAGS_ast_path + "Halt_GoAlone.json");
  halt_to_follow_ = LoadJson(FLAGS_ast_path + "Halt_Follow.json");
  halt_to_halt_ = LoadJson(FLAGS_ast_path + "Halt_Halt.json");
  halt_to_pass_ = LoadJson(FLAGS_ast_path + "Halt_Pass.json");
  pass_to_ga_ = LoadJson(FLAGS_ast_path + "Pass_GoAlone.json");
  pass_to_follow_ = LoadJson(FLAGS_ast_path + "Pass_Follow.json");
  pass_to_pass_ = LoadJson(FLAGS_ast_path + "Pass_Pass.json");
  pass_to_halt_ = LoadJson(FLAGS_ast_path + "Pass_Halt.json");
}

ast_ptr GetCond(const std::pair<string, string>& trans) {
  AST::ast_ptr pred;
  if (trans.first == "Halt" && trans.second == "GoAlone") {
    pred = halt_to_ga_;
  } else if (trans.first == "Halt" && trans.second == "Follow") {
    pred = halt_to_follow_;
  } else if (trans.first == "Halt" && trans.second == "Pass") {
    pred = halt_to_pass_;
  } else if (trans.first == "Follow" && trans.second == "Halt") {
    pred = follow_to_halt_;
  } else if (trans.first == "Follow" && trans.second == "GoAlone") {
    pred = follow_to_ga_;
  } else if (trans.first == "Follow" && trans.second == "Pass") {
    pred = follow_to_pass_;
  } else if (trans.first == "GoAlone" && trans.second == "Pass") {
    pred = ga_to_pass_;
  } else if (trans.first == "GoAlone" && trans.second == "Follow") {
    pred = ga_to_follow_;
  } else if (trans.first == "GoAlone" && trans.second == "Halt") {
    pred = ga_to_halt_;
  } else if (trans.first == "Pass" && trans.second == "Halt") {
    pred = pass_to_halt_;
  } else if (trans.first == "Pass" && trans.second == "GoAlone") {
    pred = pass_to_ga_;
  } else if (trans.first == "Pass" && trans.second == "Follow") {
    pred = pass_to_follow_;
  }
  return pred;
}

void Print(ast_ptr ast) {
    AST::Print printer;
    ast->Accept(&printer);
    printer.Display();
    cout << endl;
}

std::map<string, AST::if_ptr> InitializeBranches() {
  std::map<string, AST::if_ptr> branches;
  for (std::pair<string, string> trans : trans_list_) {
    if (branches.count(trans.first) == 0) {
      String state(trans.first);
      Dimension dims(0,0,0);
      AST::Var start("start", dims, Type::STATE);
      AST::BinOp cond(make_shared<String>(state),
                      make_shared<Var>(start), "Eq");
      ast_ptr left;
      ast_ptr right;
      AST::If branch(left, right, make_shared<BinOp>(cond));
      branches[trans.first] = make_shared<If>(branch);
    }
  }
  return branches;
}

ast_ptr BuildSocialProgram() {
  ast_ptr program;
  std::map<string, AST::if_ptr> branches = InitializeBranches();
  std::map<string, AST::if_ptr> last;

  for (const std::pair<string, string>& trans : trans_list_) {
    // Left is the output state
    AST::String left(trans.second);
    // Right is either a state or the next if block.
    ast_ptr right;
    ast_ptr cond = GetCond(trans);
    // Create a new if block
    If if_block(make_shared<AST::String>(left), right, cond);
    if (last.count(trans.first) < 1) {
      AST::if_ptr block = make_shared<If>(if_block);
      branches[trans.first]->left_ = block;
      last[trans.first] = block;
    } else {
      AST::if_ptr block = make_shared<If>(if_block);
      last[trans.first]->right_ = block;
      last[trans.first] = block;
    }
  }

  for (auto const& branch : last) {
    cout << branch.first << endl;
    AST::String left(branch.first);
    branch.second->right_ = make_shared<AST::String>(left);
    Print(branch.second);
  }

  bool started = false;
  AST::if_ptr previous;
  for (auto const& branch : branches) {
    if (!started) {
      program = branch.second;
      started = true;
    } else {
      previous->right_ = branch.second;
    }
    previous = branch.second;
  }
  String state("Failure");
  previous->right_ = make_shared<String>(state);
  cout << "-------------" << endl;
  Print(program);
  return program;
}

boost::dynamic_bitset<> ToBits(const vector<bool>& trace) {
  boost::dynamic_bitset<> bits(trace.size());
  for (size_t i = 0; i < trace.size(); ++i) {
    if (trace[i]) {
      bits[i] = 1;
    }
  }
  return bits;
}

int BestElement(const vector<boost::dynamic_bitset<>>& traces,
                boost::dynamic_bitset<> coverage) {

  auto cmp = [](std::pair<int, int> left,
                std::pair<int, int> right) {
    return left.second < right.second;
  };
  std::priority_queue<std::pair<int, int>,
                      vector<std::pair<int, int>>,
                      decltype(cmp)> queue(cmp);
  for (size_t i = 0; i < traces.size(); ++i) {
    boost::dynamic_bitset<> masked = traces[i] - coverage;
    queue.push(std::pair<int, int>(i, masked.count()));
  }
  return queue.top().first;
}

vector<json> MinCoverSet(const vector<json>& examples,
                         const vector<boost::dynamic_bitset<>>& traces) {
  // While not fully covered and growth options remain.
  boost::dynamic_bitset<> coverage(traces[0].size());
  vector<json> cover_set;
  bool updated = true;
  while (coverage.count() < coverage.size() && updated) {
    cout << coverage << endl;
    cout << "Num Examples: " << cover_set.size() << endl;
    updated = false;
    const int best = BestElement(traces, coverage);
    const boost::dynamic_bitset<> trace = traces[best];
    const json example = examples[best];
    // Update our current covering, and the set of covering examples
    const int last_score = coverage.count();
    coverage = coverage | trace;
    const int new_score = coverage.count();
    if (new_score > last_score) {
      updated = true;
      cover_set.push_back(example);
    }
  }
  cout << coverage << endl;
  cout << "Num Examples: " << cover_set.size() << endl;
  return cover_set;
}

void WriteDemos(const vector<json>& demos) {
  ofstream output_file;
  const string output_name = "covering_ex.json";
  const json output = demos;
  output_file.open(output_name);
  output_file << std::setw(4) << output << std::endl;
  output_file.close();
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  // Read in the Examples
  unordered_set<Var> variables;
  vector<std::pair<string,string>> transitions;
  vector<Example> examples = ReadExamples(FLAGS_ex_file,
      variables,
      &transitions);
  vector<json> json_ex = ReadExamples(FLAGS_ex_file);

  // Window the examples if necessary
  // examples = WindowExamples(examples, FLAGS_window_size);
  cout << "Number of Examples: " << examples.size() << endl;

  // Read in the Existing AST and the transition List.
  LoadSocialAST();

  // Use the individual conditions to build the full program.
  // (For legacy ast output).
  ast_ptr program = BuildSocialProgram();

  // Build the Set of Traces
  vector<boost::dynamic_bitset<>> trace_list;
  for (const Example& example : examples) {
    // Copy the ast
    ast_ptr p_copy = AST::DeepCopyAST(program);
    // const auto& example = examples[0];
    AST::Interpret(p_copy, example);

    // Extract the trace information as a vector
    cout << " ----------" << endl;
    boost::dynamic_bitset<> trace = ToBits(AST::TraceOn(p_copy));
    trace_list.push_back(trace);
    cout << " ----------" << endl;

    if (kDebug) {
      cout << trace << endl;
    }
  }
  vector<json> cover_set = MinCoverSet(json_ex, trace_list);

  // Dump the minimal covering set to a file
  WriteDemos(cover_set);
}
