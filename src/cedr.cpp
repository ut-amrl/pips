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

DEFINE_string(ex_file, "merged.json", "Examples file");
DEFINE_string(small_ex_file, "covering_ex.json", "Examples file");
DEFINE_string(lib_file, "ops/social_ref.json", "Operation library file");
DEFINE_string(ast_path, "cedrF/policies/ldips4/", "Operation library file");
DEFINE_uint32(feat_depth, 2, "Maximum enumeration depth for features.");
DEFINE_uint32(sketch_depth, 2, "Maximum enumeration depth for sketch.");
DEFINE_uint32(window_size, 0, "Size of sliding window to subsample demonstrations with.");
DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");
DEFINE_bool(debug, true, "Enable Debug Printing");

using AST::ast_ptr;
using AST::BinOp;
using AST::If;
using AST::if_ptr;
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
using boost::dynamic_bitset;
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
using std::pair;
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

const bool kDebug = false;
int num_actions_ = 0;

void WriteDemos(const vector<json>& demos) {
  ofstream output_file;
  const string output_name = "big_cover.json";
  const json output = demos;
  output_file.open(output_name);
  output_file << std::setw(4) << output << std::endl;
  output_file.close();
}

vector<std::pair<string,string>> LoadTransitionList(const string& path) {
  std::ifstream infile(path);
  std::set<string> actions;
  vector<std::pair<string, string>> trans_list;
  string start, output;
  while(infile >> start >> output) {
    const std::pair<string, string> trans(start, output);
    trans_list.push_back(trans);
    actions.insert(start);
    actions.insert(output);
  }
  num_actions_ = actions.size();
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

AST::bin_ptr GetCond(const std::pair<string, string>& trans) {
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
  return std::dynamic_pointer_cast<BinOp>(pred);
}

void Print(ast_ptr ast) {
    AST::Print printer;
    ast->Accept(&printer);
    printer.Display();
    cout << endl;
}

std::map<string, if_ptr> InitializeBranches() {
  std::map<string, if_ptr> branches;
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

// TODO(jaholtz) clean up how this if branching is displayed.
ast_ptr BuildIf(const string& state, AST::bin_ptr cond, ast_ptr next) {
  ast_ptr next_copy = AST::DeepCopyAST(next);
  ast_ptr left;
  ast_ptr right;
  ast_ptr condition;
  if (cond->op_ == "&&") {
    left = BuildIf(state,
        std::dynamic_pointer_cast<AST::BinOp>(cond->right_), next_copy);
    condition = cond->left_;
    right = next;
  } else if (cond->op_ == "||") {
    AST::String left_str(state);
    left = make_shared<AST::String>(state);
    right = BuildIf(state,
        std::dynamic_pointer_cast<AST::BinOp>(cond->right_), next_copy);
    condition = cond->left_;
  } else { // There is only one condition here, no need to expand
    // Left is the output state
    AST::String left_str(state);
    left = make_shared<AST::String>(state);
    // Right is either a state or the next if block.
    right = next_copy;
    condition = std::dynamic_pointer_cast<BinOp>(cond);
  }
  If if_block(left, right, condition);
  return make_shared<If>(if_block);
}

ast_ptr BuildSocialProgram(const bool& sketch) {
  ast_ptr program;
  std::map<string, if_ptr> branches = InitializeBranches();
  std::map<string, if_ptr> last;

  ast_ptr last_block;
  int state_count = 0;

  for (const std::pair<string, string>& trans : trans_list_) {
    cout << trans.first << " --> " << trans.second << endl;
    AST::bin_ptr cond = GetCond(trans);
    if (sketch) {
      Feature blank_feat("blank", {0,0,0}, NUM);
      ast_ptr blank_ptr = make_shared<Feature>(blank_feat);
      AST::BinOp blank(blank_ptr, blank_ptr, "");
      cond = make_shared<BinOp>(blank);
    }
    ast_ptr if_block;
    if (state_count == 0) {
      String state(trans.first);
      ast_ptr same_block = make_shared<String>(state);
      if_block = BuildIf(trans.second, cond, same_block);
      if (trans.first == "Pass") {
        state_count += 1;
      } else if (trans.first == "Follow") {
        state_count += 1;
      } else if (trans.first =="Halt") {
        state_count += 0;
      } else if (trans.first == "GoAlone") {
          // state_count += 1;
      }
    } else {
      if_block = BuildIf(trans.second, cond, last_block);
    }
    last_block = if_block;
    state_count++;
    cout << "Num Actions: " << num_actions_ << endl;
    if (state_count == num_actions_ - 1) {
      state_count = 0;
      cout << "adding branch" << endl;
      branches[trans.first]->left_ = if_block;
    }
  }

  bool started = false;
  if_ptr previous;
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
  return program;
}

vector<Example> FindFailing(const vector<Example>& examples, ast_ptr program) {
  vector<Example> failing_examples;
  for (size_t i = 0; i < examples.size(); ++i) {
    const Example example = examples[i];
    ast_ptr p_copy = AST::DeepCopyAST(program);
    ast_ptr result = AST::Interpret(p_copy, example);
    AST::string_ptr result_string =
        std::dynamic_pointer_cast<String>(result);
    if (example.result_.GetString() != result_string->value_) {
      // cout << "Expected: " << example.result_.GetString() << endl;
      // cout << "Returned: " << result_string->value_ << endl;
      failing_examples.push_back(example);
    }
  }
  return failing_examples;
}

vector<Example> FindSucceeding(const vector<Example>& examples, ast_ptr program) {
  vector<Example> failing_examples;
  for (size_t i = 0; i < examples.size(); ++i) {
    const Example example = examples[i];
    ast_ptr p_copy = AST::DeepCopyAST(program);
    ast_ptr result = AST::Interpret(p_copy, example);
    AST::string_ptr result_string =
        std::dynamic_pointer_cast<String>(result);
    if (example.result_.GetString() == result_string->value_) {
      // cout << "Expected: " << example.result_.GetString() << endl;
      // cout << "Returned: " << result_string->value_ << endl;
      failing_examples.push_back(example);
    }
  }
  return failing_examples;
}

// Identifies all input program candidates that are not consistent with the
// input example.
dynamic_bitset<> TestCandidates(const Example& example,
                              const vector<ast_ptr>& programs) {
  dynamic_bitset<> failing_examples(programs.size());
  for (size_t i = 0; i < programs.size(); ++i) {
    ast_ptr program = programs[i];
    ast_ptr p_copy = AST::DeepCopyAST(program);
    ast_ptr result = AST::Interpret(p_copy, example);
    AST::string_ptr result_string =
      std::dynamic_pointer_cast<String>(result);
    if (example.result_.GetString() != result_string->value_) {
      failing_examples[i] = 1;
    }
  }
  return failing_examples;
}

// Identifies all input program candidates that are not consistent with the
// input example.
dynamic_bitset<> TestCandidates(const Example& example,
                                const bool& result,
                                const vector<ast_ptr>& programs) {
  dynamic_bitset<> failing_examples(programs.size());
  for (size_t i = 0; i < programs.size(); ++i) {
    ast_ptr program = programs[i];
    ast_ptr p_copy = AST::DeepCopyAST(program);
    bool interp_result = AST::InterpretBool(p_copy, example);
    if (interp_result != result) {
      failing_examples[i] = 1;
    }
  }
  return failing_examples;
}


int BestExampleIndex(const vector<int>& scores) {
  auto cmp = [](std::pair<int, int> left,
                std::pair<int, int> right) {
    return left.second < right.second;
  };
  std::priority_queue<std::pair<int, int>,
                      vector<std::pair<int, int>>,
                      decltype(cmp)> queue(cmp);
  for (size_t i = 0; i < scores.size(); ++i) {
    queue.push(std::pair<int, int>(i, scores[i]));
  }
  const pair<int,int> top = queue.top();
  cout << "Best Example Score: " << top.second << endl;
  if (queue.size() == 0) {
    return 0;
  }
  if (top.second == 0) {
    return -1;
  }
  return queue.top().first;
}

// Identifies the index of the example that most significantly reduces the
// search space.
int GetCounterExampleIndex(vector<Example>& examples,
                           const vector<vector<ast_ptr>>& programs) {
  vector<int> example_scores;
  #pragma omp parallel
  for (const Example& example : examples) {
    int score = 0;
    bool bad_example = false;
    for (size_t i = 0; i < trans_list_.size(); ++i) {
      const pair<string, string> transition = trans_list_[i];
      const string out = transition.second;
      const string in = transition.first;
      const vector<ast_ptr> trans_programs = programs[i];
      bool result = false;
      if (example.result_ == out && example.start_ == in) {
        result = true;
      }
      const int failing_examples =
          TestCandidates(example, result, trans_programs).count();
      if (failing_examples == static_cast<int>(trans_programs.size())) {
        bad_example = true;
      }
      score += failing_examples;
    }
    #pragma omp critical
    {
      if (bad_example) {
        example_scores.push_back(-1);
      } else {
        example_scores.push_back(score);
      }
    }
  }
  vector<Example> pruned_examples;
  vector<int> pruned_scores;
  for (size_t i = 0; i < examples.size(); ++i) {
    if (example_scores[i] > 0) {
      pruned_examples.push_back(examples[i]);
      pruned_scores.push_back(example_scores[i]);
    }
  }
  examples = pruned_examples;
  example_scores = pruned_scores;
  const int best_index = BestExampleIndex(example_scores);
  return best_index;
}

int GetCounterExampleIndex(vector<Example>& examples,
                           const vector<ast_ptr>& programs,
                           const pair<string, string>& transition) {
  vector<int> example_scores(examples.size());
  int count = 0;
  cout << "Test Examples" << endl;
  #pragma omp parallel
  for (size_t i = 0; i < examples.size(); ++i) {
    const Example& example = examples[i];
    bool bad_example = false;
    const string out = transition.second;
    const string in = transition.first;
    bool result = false;
    if (example.start_.GetString() != in) {
      example_scores[i] = 0;
    } else {
      if (example.result_ == out) {
        result = true;
      }
      const int failing_examples =
        TestCandidates(example, result, programs).count();
      if (failing_examples == static_cast<int>(programs.size())) {
        // bad_example = true;
      }
      // cout << "Examples Considered: " << count << endl;
      count++;
      if (bad_example) {
        example_scores[i] = -1;
      } else {
        example_scores[i] = failing_examples;
      }
    }
  }
  // cout << "Prune Examples" << endl;
  // cout << "Programs: " << programs.size() << endl;
  vector<Example> pruned_examples;
  vector<int> pruned_scores;
  for (size_t i = 0; i < examples.size(); ++i) {
    if (example_scores[i] >= 0) {
      pruned_examples.push_back(examples[i]);
      pruned_scores.push_back(example_scores[i]);
    }
  }
  examples = pruned_examples;
  example_scores = pruned_scores;
  const int best_index = BestExampleIndex(example_scores);
  return best_index;
}

vector<Example> GetRelevantExamples(const vector<Example>& examples,
                                    const vector<pair<string, string>>& trans,
                                    const int& current_trans) {
  vector<Example> partial_example = examples;
  for (int i = 0; i < current_trans; ++i) {
    const pair<string, string> transition = trans[i];
    partial_example = FilterExamples(partial_example, transition);
  }
  return partial_example;
}

// Find the best example, and remove it, along with any compliant examples.
Example CoverageCEX(ast_ptr program,
                    vector<Example>& examples) {
  Example counter;
  return counter;
}

void NoSearchCedr(ast_ptr program,
                  const vector<Example>& examples,
                  vector<Example>& rehearsed) {
  vector<Example> remaining = examples;

  // While there are any remaining examples that disagree with the program
  while (remaining.size() > 0) {
    // Synthesize a program given the rehearsed set

    // Read in the Existing AST and the transition List.
    LoadSocialAST();
    std::reverse(trans_list_.begin(), trans_list_.end());
    ast_ptr program = BuildSocialProgram(false);

    // Find the best counter example for the current program and remove
    // any compliant examples.
    const Example counter = CoverageCEX(program, remaining);
    rehearsed.push_back(counter);

    cout << "++++++++" << endl;
    cout << rehearsed.size() << endl;
    cout << remaining.size() << endl;
    Print(program);
    cout << "++++++++" << endl;
  }
}

vector<json> GetJsonExamples(const vector<Example>& examples,
                             const vector<json>& json_ex) {
  vector<json> subsampled;
  for (const Example& example : examples) {
    subsampled.push_back(json_ex[example.index]);
  }
  return subsampled;
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  // Read in the Examples
  unordered_set<Var> variables;
  vector<std::pair<string,string>> transitions;
  vector<Example> examples = ReadExamples(FLAGS_ex_file,
      variables,
      &transitions);
  vector<json> large_json_ex = ReadExamples(FLAGS_ex_file);

  unordered_set<Var> start_variables;
  vector<std::pair<string,string>> start_transitions;
  vector<Example> start_examples = ReadExamples(FLAGS_small_ex_file,
      start_variables,
      &start_transitions);
  vector<json> json_ex = ReadExamples(FLAGS_small_ex_file);

  // Turning variables into roots
  vector<ast_ptr> inputs, roots;
  for (const Var& variable : variables) {
    if (variable.name_ != "goal" && variable.name_ != "free_path" &&
        variable.name_ != "DoorState" && variable.name_ != "DoorPose") {
      roots.push_back(make_shared<Var>(variable));
    }
  }
  inputs = roots;

  // Read in the Existing AST and the transition List.
  LoadSocialAST();

  cout << "Building Program" << endl; // Use the individual conditions to build the full program.
  // (For legacy ast output).
  // Reverse the list, working backwards
  std::reverse(trans_list_.begin(), trans_list_.end());
  ast_ptr program = BuildSocialProgram(false);

  Print(program);

  // Run each demonstration on the program, and store all of the failing ones
  // in a separate vector.
  vector<Example> failing = FindFailing(examples, program);
  const vector<Example> succeeding = FindSucceeding(examples, program);

  // For now just print the count at the end (we can use that to compare).
  cout << "Failing Demos: " << failing.size() << endl;
  const float success_rate =
      (examples.size() - failing.size());
  cout << "Suceeding Demos: " << success_rate << endl;
  cout << "Succeeding Examples: ";
  cout << float(success_rate / float(examples.size())) << endl;
  // Empty out all of the predicates in the sketch
  // (to create a blank if/else sketch)
  cout << "Building Blank Program" << endl;
  // ast_ptr sketch = BuildSocialProgram(true);
  // Print(sketch);

  // Loading Library Function Definitions
  vector<FunctionEntry> library = ReadLibrary(FLAGS_lib_file);

  // Enumerate features up to a fixed depth
  vector<Signature> signatures;
  vector<ast_ptr> ops = AST::RecEnumerate(roots, inputs, examples, library,
                                          FLAGS_feat_depth, &signatures);
  examples = succeeding;

  cout << "Features enumerated" << endl;

  vector<ast_ptr> sketches;
  // AST::EnumerateL3(FLAGS_sketch_depth, ops, sketch, sketches);
  vector<vector<ast_ptr>> remaining_sketches;
  vector<vector<ast_ptr>> comp_sketches;
  // TODO(jaholtz) Identify a way to do this enumeration incrementally
  // Enumerate all candidates maybe, only solve some of them.
  // Push back the depths.
  EnumLdipsL3(start_examples,
              trans_list_,
              ops,
              FLAGS_sketch_depth,
              1.0,
              &remaining_sketches,
              &comp_sketches);

  cout << "Sketches Enumerated" << endl;
  for (size_t i = 0; i < remaining_sketches.size(); ++i) {
    cout << "Number of Sketches for Transition " << i << " : ";
    cout << remaining_sketches[i].size() << endl;
  }

  cout << "Updating Frontier" << endl;
  const int kFrontierSize = 500;
  vector<vector<ast_ptr>> cand_sketches(remaining_sketches.size());
  comp_sketches.resize(remaining_sketches.size());
  UpdateFrontier(start_examples,
                trans_list_,
                1.0,
                kFrontierSize,
                &remaining_sketches,
                &cand_sketches,
                &comp_sketches);

  cout << "Frontiers Updated" << endl;

  // Reverse the list again to work from top to bottom.
  std::reverse(trans_list_.begin(), trans_list_.end());

  for (size_t j = trans_list_.size(); j > 0; --j) {
    const int i = j - 1;
    cout << endl;
    cout << i << endl;
    cout << "=== Transition: " << trans_list_[i].first << "-->";
    cout << trans_list_[i].second << " ===" << endl;
    vector<Example> relevant_examples =
        GetRelevantExamples(examples, trans_list_, i);
    const vector<Example> all_relevant = relevant_examples;
    bool finished = false;
    while (cand_sketches[i].size() > 1 &&
           relevant_examples.size() > 0) {
      // Find a counter example from the set of examples
      const int counter_index = GetCounterExampleIndex(relevant_examples,
                                                       comp_sketches[i],
                                                       trans_list_[i]);
      cout << "---- Selected Counter Example -----" << endl;
      cout << "Reamaining Examples: " << relevant_examples.size() << endl;
      cout << "Collected Examples: " << start_examples.size() << endl;
      if (counter_index == -1) {
        finished = true;
      } else {
        const Example counter = relevant_examples[counter_index];
        start_examples.push_back(counter);
        relevant_examples.erase(relevant_examples.begin() + counter_index);
      }

      // Resynth every candidate and keep matching ones.
      if (finished) {
        cand_sketches[i].clear();
        comp_sketches[i].clear();
        finished = false;
      } else {
        PruneL2(start_examples, trans_list_, 1.0, &cand_sketches, &comp_sketches);
      }
      UpdateFrontier(start_examples,
              trans_list_,
              1.0,
              kFrontierSize,
              &remaining_sketches,
              &cand_sketches,
              &comp_sketches);
      cout << "---- Pruned programs -----" << endl;
      cout << "Frontier Size: " << cand_sketches[i].size() << endl;
      cout << "Horizon Size: " << remaining_sketches[i].size() << endl;
      cout << "---- ----- -----" << endl;

    }
    cout << "---All Final Sketches---" << endl;
    vector<json> output_vec;
    // for (ast_ptr sketch : comp_sketches[i]) {
        // cout << "Sketches Remaining: " << comp_sketches[i].size() << endl;
        // // Print(sketch);
        // float pos, neg;
        // float score =
            // AST::ScorePredicate(sketch,
                                // trans_list_[i],
                                // all_relevant,
                                // &pos,
                                // &neg);
        // // cout << "Percentage Satisfied: " << score << endl;
        // const json json_out = sketch->ToJson();
        // output_vec.push_back(json_out);
    // }
    ofstream output_file;
    const string output_folder = "synthd/cedr/";
    const string output_name =
        output_folder + trans_list_[i].first + "_" + trans_list_[i].second + ".txt";
    output_file.open(output_name);
    json output = output_vec;
    output_file << std::setw(4) << output << std::endl;
    output_file.close();
  }
  cout << "Writing New Demonstrations to File" << endl;
  // Dump the minimal covering set to a file
  const vector<json> subsampled_json =
      GetJsonExamples(start_examples, large_json_ex);
  WriteDemos(subsampled_json);
}
