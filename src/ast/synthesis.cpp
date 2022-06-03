#include "synthesis.hpp"

#include <gflags/gflags.h>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <queue>

#include "enumeration.hpp"
#include "gflags/gflags_declare.h"
#include "parsing.hpp"
#include "visitors/deepcopy_visitor.hpp"
#include "visitors/fillhole_visitor.hpp"
#include "visitors/interp_visitor.hpp"
#include "visitors/print_visitor.hpp"
#include "visitors/tosmtlib_visitor.hpp"
#include "../submodules/amrl_shared_lib/util/timer.h"
#include "utils/nd_bool_array.hpp"

using std::vector;
using std::pair;
using std::string;
using std::cout;
using std::endl;
using std::ofstream;
using std::unordered_map;
using std::unordered_set;
using std::make_shared;
using nlohmann::json;
using AST::CheckModelAccuracy;

DECLARE_uint32(sketch_depth);
DECLARE_double(min_accuracy);
DECLARE_bool(debug);

namespace AST {

// Basically a breadth-first search of all combination of indices of the ops
// vector starting at {0, 0, ..., 0}
class index_iterator {
 public:
  index_iterator(size_t op_count, size_t holes_to_fill)
      : op_count_(op_count),
        holes_to_fill_(holes_to_fill),
        visited_(
            nd_bool_array(vector<size_t>(holes_to_fill_, op_count_))) {  // :^)
    // Some checks to make sure what we're doing is sensible.
    if (op_count_ == 0) {
      throw std::invalid_argument("no ops!");
    }
    if (holes_to_fill_ == 0) {
      throw std::invalid_argument("no holes to fill!");
    }

    // make the initial list of however many 0s
    vector<size_t> initial_coords;
    for (size_t i = 0; i < holes_to_fill_; ++i) {
      initial_coords.push_back(0);
    }

    // Add it to the queue to start BFS
    indicies_.push(initial_coords);
  }

  vector<size_t> next() {
    // Get the current lowest priority index list
    vector<size_t> current = indicies_.front();
    indicies_.pop();

    // Add neighbors of this index list to the queue
    for (size_t i = 0; i < holes_to_fill_; ++i) {
      // if we've reached an edge, don't do anything
      if (current[i] == op_count_ - 1) {
        continue;
      }

      // Copy current index list, increase one index by 1, add it to the queue,
      // and mark it as visited so it's not added again.
      vector<size_t> copy = current;
      copy[i] += 1;
      if (!visited_.get(copy)) {
        indicies_.push(copy);
        visited_.set(copy, true);
      }
    }
    return current;
  }

  bool has_next() const { return !indicies_.empty(); }

  vector<size_t> zeros() const { return vector<size_t>(holes_to_fill_, 0); }

 private:
  const size_t op_count_;
  const size_t holes_to_fill_;
  std::queue<vector<size_t>> indicies_;
  nd_bool_array visited_;
};

// Helper Function for scoring a candidate predicate given only a set
// of examples and the desired transition
float ScorePredicate(ast_ptr pred, const pair<string, string>& transition,
    const vector<Example>& examples, float* pos, float* neg) {
  // Checking Initial accuracy
  const SymEntry out(transition.second);
  const SymEntry in(transition.first);

  // Split up all the examples into a "yes" set or "no" sets
  unordered_set<Example> yes;
  unordered_set<Example> no;
  SplitExamples(examples, transition, &yes, &no);
  return CheckModelAccuracy(pred, yes, no, pos, neg);
}

// Utility function that converts a Z3 model (solutions for parameter holes)
// into a C++ unordered_map associating the names of parameter holes with
// floats.
Model Z3ModelToMap(const z3::model& z3_model) {
  Model ret;
  for (size_t i = 0; i < z3_model.size(); ++i) {
    const z3::func_decl v = z3_model[i];
    const z3::expr interp = z3_model.get_const_interp(v);
    if (!interp.is_numeral()) {
      throw std::invalid_argument("All model values must be numeral");
    }
    // Keep these as int64_t, otherwise you might get overflow errors and Z3
    // will complain.
    const int64_t numerator = interp.numerator().get_numeral_int64();
    const int64_t denominator = interp.denominator().get_numeral_int64();
    const float value = (float)numerator / denominator;
    const string key = v.name().str();
    Num value_ast(value, {0, 0, 0});
    ret[key] = make_shared<Num>(value_ast);
  }
  return ret;
}

CumulativeFunctionTimer solve_smtlib("SolveSMTLIBProblem");
Model SolveSMTLIBProblem(const string& problem) {
  CumulativeFunctionTimer::Invocation invoke(&solve_smtlib);
  z3::context context;
  z3::optimize solver(context);
  solver.from_string(problem.c_str());
  z3::check_result result = solver.check();
  if (result == z3::sat) {
    z3::model m = solver.get_model();
    return Z3ModelToMap(m);
  } else {
    throw std::invalid_argument("UNSAT");
  }
}

CumulativeFunctionTimer make_smtlib("MakeSMTLIBProblem");
string MakeSMTLIBProblem(const unordered_set<Example>& yes,
                         const unordered_set<Example>& no,
                         const ast_ptr program,
                         const bool soft,
                         const bool srtr) {
  CumulativeFunctionTimer::Invocation invoke(&make_smtlib);

  // Create empty sets to hold data needed for program generation. The params
  // set will track the names of parameter holes discovered, and the assertions
  // set will hold the assertions we generate.
  unordered_set<string> params;
  vector<string> assertions;

  // Make a new set that includes all examples.
  unordered_set<Example> all_examples(yes);
  all_examples.insert(no.cbegin(), no.cend());
  // For every example, ...
  for (const Example& example : yes) {
    // Partially evaluate the program with the examples as much as possible.
    ast_ptr partial = Interpret(program, example);
    // Create an SMT-LIB expression from the program and that example.
    ToSMTLIB converter = AstToSMTLIB(partial, example);
    const string smtlib = converter.Get();
    // Add full assertions using that expression to our assertion list. It's
    // important to check whether the example is in both sets because we could
    // have contradictory examples.
    if (soft) {
      assertions.push_back("(assert-soft " + smtlib + ")");
    }
    else {
      assertions.push_back("(assert " + smtlib + ")");
    }
    // If there are any parameter holes, make note of them too.
    const unordered_set<string> example_params = converter.GetParams();
    params.insert(example_params.cbegin(), example_params.cend()); 
  }

  for (const Example& example : no) {
    // Partially evaluate the program with the examples as much as possible.
    ast_ptr partial = Interpret(program, example);
    // Create an SMT-LIB expression from the program and that example.
    ToSMTLIB converter = AstToSMTLIB(partial, example);
    const string smtlib = converter.Get();
    // Add full assertions using that expression to our assertion list. It's
    // important to check whether the example is in both sets because we could
    // have contradictory examples.
    if (soft) {
      assertions.push_back("(assert-soft (not " + smtlib + "))");
    }
    else {
      assertions.push_back("(assert (not " + smtlib + "))");
    }
    // If there are any parameter holes, make note of them too.
    const unordered_set<string> example_params = converter.GetParams();
    params.insert(example_params.cbegin(), example_params.cend()); 
  }

  string problem = "";

  // For every parameter hole discovered, add a real constant to the problem.
  for (const string& param : params) {
    problem += "(declare-const " + param + " Real)\n";
  }

  // Add every assertion created previously to the problem.
  for (const string& assertion : assertions) {
    problem += assertion + "\n";
  }

  if (srtr) {
    for (const string& param : params) {
      // minimize the absolute value of these constants. Closer to 0 is better
      // in the SRTR case, and ignoreable the rest of the time (because of
      // lexicographic optimization)
      const string absolute =
          "(ite (> 0 " + param + ") (- 0 " + param + ") " + param + ")";
      problem += "(minimize " + absolute + ")\n";
    }
  }

  // Now that the problem string has been completely generated, return it.
  // There is no need to add instructions to the solver like (check-sat) or
  // (get-model) since those actions will be performed by us using the Z3 C++
  // API later.
  return problem;
}

// Attempts to solve for a satisfactory assignment of real values to
// parameter holes that satisfies a subset of examples. Returns the
// percentage of examples satisfied.
double PredicateL1(ast_ptr sketch, const unordered_set<Example>& pos,
    const unordered_set<Example>& neg, const bool srtr) {
  const string problem = MakeSMTLIBProblem(pos, neg, sketch, true, srtr);
  try {
    const Model solution = SolveSMTLIBProblem(problem);
    if (solution.empty()) {
      // throw std::invalid_argument("UNSAT");
    }
    FillHoles(sketch, solution);
    const double output = CheckModelAccuracy(sketch, pos, neg);
    if (FLAGS_debug) {
      cout << "L1 Candidate: " << sketch << endl;
      cout << "Score : " << output << endl << endl;
    }
    return output;
  } catch (const std::invalid_argument) {
    return 0.0;
  }
}

void UpdateRuleProblem(z3::optimize& solver, z3::context& context, ast_ptr program, const vector<Example>& observation) {
  // Create empty sets to hold data needed for program generation. The params
  // set will track the names of parameter holes discovered.
  unordered_set<string> params;
  string smt_string = "";
  for (const Example& example : observation) {
    // Partially evaluate the program with the examples as much as possible.
    ast_ptr partial = Interpret(program, example);
    // Create an SMT-LIB expression from the program and that example.
    ToSMTLIB converter = AstToSMTLIB(partial, example);
    const string smtlib = converter.Get();
    // Add full assertions using that expression to our assertion list. It's
    // important to check whether the example is in both sets because we could
    // have contradictory examples.
    smt_string += "" + smtlib + "\n";
    
    // If there are any parameter holes, make note of them too.
    const unordered_set<string> example_params = converter.GetParams();
    params.insert(example_params.cbegin(), example_params.cend());
  }
  string new_clause = "";
  for (const string& param : params) {
    new_clause += "(declare-const " + param + " Real)\n";
  }
  new_clause += "(assert (or " + smt_string + "))\n";
  solver.push();
  auto expr = context.parse_string(new_clause.c_str());
  solver.add(expr);
}

double TrapL1(ast_ptr sketch, const vector<vector<vector<Example>>>& positive, const vector<vector<Example>>& negative) {
  // Flatten the 'positive' examples and form an SMT-problem the solution which should return false for
  // all positive examples. 
  std::unordered_set<Example> positives;
  for (const auto& trajectory : positive) {
    for (const auto& observation : trajectory) {
      for (const Example& example : observation) {
        positives.insert(example);
      } 
    }
  }
  // TODO(jaholtz) Relax this constraint.
  const string problem = MakeSMTLIBProblem({}, positives, sketch, false, false);
  z3::context context;
  z3::optimize solver(context);
  solver.from_string(problem.c_str());
  z3::check_result result = solver.check();
  if (result != z3::sat) {
    return 0;
  }
  
  // For each step in the negative trajectory add a clause and solve iteratively. Record the length of the sequence identified and the final rule.
  auto solution = Z3ModelToMap(solver.get_model());
  int output = 0;
  for (size_t i = 0; i < negative.size(); ++i) {
    const vector<Example> observation = negative[i];
    // Update the solver
    UpdateRuleProblem(solver, context, sketch, observation);
    z3::check_result result = solver.check();
    if (result != z3::sat) {
      break;
    }
    output = i + 1;
    solution = Z3ModelToMap(solver.get_model());
  }
  cout << "L1 Candidate: " << sketch << endl;
  cout << "Trap Size: " << output << endl << endl;
  FillHoles(sketch, solution);
  return output;
}

ast_ptr FillFeatureHoles(ast_ptr sketch, const vector<size_t>& indicies,
    const vector<ast_ptr>& ops) {
  Model m;

  // Get a list of the names of all the feature holes in the conditional,
  // then store them in a vector because being able to access them in a
  // consistent order and by index is important for something we do later.
  const unordered_map<string, pair<Type, Dimension>> feature_hole_map =
    MapFeatureHoles(sketch);
  vector<string> feature_holes;
  for (const auto& p : feature_hole_map) {
    feature_holes.push_back(p.first);
  }
  const size_t feature_hole_count = feature_holes.size();

  // For every feature hole...
  for (size_t i = 0; i < feature_hole_count; ++i) {
    // Get the name of a feature hole to fill and a possible value for it.
    const string& feature_hole = feature_holes[i];
    const pair<Type, Dimension> feature_hole_info =
      feature_hole_map.at(feature_hole);
    const Type feature_hole_type = feature_hole_info.first;
    const Dimension feature_hole_dims = feature_hole_info.second;
    const size_t index = indicies[i];
    const ast_ptr& op = ops[index];
    // If the type of the selected op doesn't match, we can stop creating
    // this model. Otherwise we added this op to the model.
    if (op->type_ != feature_hole_type) {
      break;
    } else {
      // const ast_ptr op_copy = DeepCopyAST(op);
      m[feature_hole] = op;
    }
  }

  // If after creating the model the number of filled holes is not the same
  // as the number of holes that ought to be filled (indicating a type
  // error), we can give up and try the next model.
  if (m.size() != feature_hole_count) {
    return nullptr;
  }

  // Since no errors occured while creating the model, we can take the hole
  // values from it and use them to fill the holes in a copy of the
  // condition.
  ast_ptr filled = DeepCopyAST(sketch);
  FillHoles(filled, m);
  return filled;
}

CumulativeFunctionTimer trap_l2("TrapL2");
ast_ptr TrapL2(const vector<vector<Example>>& neg_example, 
               const vector<vector<vector<Example>>>& positive, ast_ptr sketch,
               const vector<ast_ptr>& ops,
               const int min_trap,  float* solved) {
  CumulativeFunctionTimer::Invocation invoke(&trap_l2);
  // Get a list of the names of all the feature holes in the conditional,
  // then store them in a vector because being able to access them in a
  // consistent order and by index is important for something we do later.
  const unordered_map<string, pair<Type, Dimension>> feature_hole_map =
      MapFeatureHoles(sketch);
  vector<string> feature_holes;
  for (const auto& p : feature_hole_map) {
    feature_holes.push_back(p.first);
  }
  const size_t feature_hole_count = feature_holes.size();
  
  if (FLAGS_debug) {
    cout << "Current Sketch: " << sketch << endl;
  }
  
  ast_ptr solution_cond = sketch;
  float current_best =  *solved;
  index_iterator c(ops.size(), feature_hole_count);
  bool keep_searching = true;
  int count = 0.0;
  #pragma omp parallel
  while (keep_searching) {
    // Use the indices given to us by the iterator to select ops for filling
    // our feature holes and create a model.
    vector<size_t> op_indicies;
    #pragma omp critical
    {
      if (c.has_next()) {
        op_indicies = c.next();
      } else {
        keep_searching = false;
        op_indicies = c.zeros();
      }
      count++;
    }
    ast_ptr filled = FillFeatureHoles(sketch, op_indicies, ops);
    if (filled != nullptr) {
      // TODO(jaholtz) Check if a solution exists
      #pragma omp critical
      {
        const int trap_size = TrapL1(filled, positive, neg_example);
        if (keep_searching && trap_size >= min_trap) {
          keep_searching = false;
          solution_cond = filled;
          current_best = trap_size;
        } else if (trap_size > current_best) {
          solution_cond = filled;
          current_best = trap_size;
        }
      }
    }
  }
  *solved = current_best;
  return solution_cond;
}

CumulativeFunctionTimer pred_l2("PredicateL2");
ast_ptr PredicateL2(
    const vector<Example>& examples, const vector<ast_ptr>& ops,
    ast_ptr sketch, const pair<string,string>& transition,
    const double min_accuracy, float* solved) {
  CumulativeFunctionTimer::Invocation invoke(&pred_l2);

  const SymEntry out(transition.second);
  const SymEntry in(transition.first);

  // Get a list of the names of all the feature holes in the conditional,
  // then store them in a vector because being able to access them in a
  // consistent order and by index is important for something we do later.
  const unordered_map<string, pair<Type, Dimension>> feature_hole_map =
      MapFeatureHoles(sketch);
  vector<string> feature_holes;
  for (const auto& p : feature_hole_map) {
    feature_holes.push_back(p.first);
  }
  const size_t feature_hole_count = feature_holes.size();

  // Split up all the examples into a "yes" set or a "no" set based on
  // whether the result for the example matches the current example's
  // behavior.
  unordered_set<Example> yes;
  unordered_set<Example> no;
  SplitExamples(examples, transition, &yes, &no);

  if (FLAGS_debug) {
    cout << "Current Sketch: " << sketch << endl;
  }

  // Start iterating through possible models. index_iterator is explained
  // seperately.
  ast_ptr solution_cond = sketch;
  float current_best =  0.0;
  if (feature_hole_count > 0) {
    index_iterator c(ops.size(), feature_hole_count);
    solution_cond = nullptr;
    bool keep_searching = true;
    int count = 0.0;
    #pragma omp parallel
    while (keep_searching) {
      // Use the indices given to us by the iterator to select ops for filling
      // our feature holes and create a model.
      vector<size_t> op_indicies;
      #pragma omp critical
      {
        if (c.has_next()) {
          op_indicies = c.next();
        } else {
          keep_searching = false;
          op_indicies = c.zeros();
        }
        count++;
      }

      ast_ptr filled = FillFeatureHoles(sketch, op_indicies, ops);
      if (filled != nullptr) {
        const double sat_ratio = PredicateL1(filled, yes, no, false);
        #pragma omp critical
        {
          if (keep_searching && sat_ratio >= min_accuracy) {
            keep_searching = false;
            solution_cond = filled;
            current_best = sat_ratio;
          } else if (sat_ratio >= current_best) {
            solution_cond = filled;
            current_best = sat_ratio;
          }
        }
      }
    }
  } else {
    // Since no errors occured while creating the model, we can take the hole
    // values from it and use them to fill the holes in a copy of the
    // condition.
    ast_ptr cond_copy = DeepCopyAST(sketch);
    const double sat_ratio = PredicateL1(cond_copy, yes, no, false);
    #pragma omp critical
    {
      if (sat_ratio >= current_best) {
        solution_cond = cond_copy;
        current_best = sat_ratio;
      }
    }
  }

  // Return the solution if one exists, otherwise return a nullptr.
  if (solution_cond != nullptr) {
    *solved = current_best;
  }
  return solution_cond;
}

ast_ptr ldipsL2(ast_ptr candidate,
    const vector<Example>& examples,
    const vector<ast_ptr>& ops,
    const pair<string, string>& transition,
    const float min_accuracy,
    ast_ptr best_program,
    float* best_score) {

  // Find best performing completion of current sketch
  float solved = 0.0;
  ast_ptr solution =
    PredicateL2(examples,
        ops, candidate, transition, min_accuracy, &solved);

  // Keep if beats best performing solution.
  if (solved > *best_score) {
    *best_score = solved;
    best_program = solution;
  }
  Z3_reset_memory();
  return best_program;
}

vector<ast_ptr> PruneRuleOps(vector<ast_ptr> ops, vector<vector<Example>>& neg) {
  vector<ast_ptr> pruned;
  vector<Example> observation = neg[0];
  for (const ast_ptr& op : ops) {
    // Only keep ops that are true for at least the first observation
    z3::context context;
    z3::optimize solver(context);
    UpdateRuleProblem(solver, context, op, observation);
    z3::check_result result = solver.check();
    if (result == z3::sat) {
      pruned.push_back(op);
    }
  }
  return pruned;
}

ast_ptr RuleL2(const vector<Example>& neg_example, 
               const vector<Example>& positive, ast_ptr sketch,
               const vector<ast_ptr>& ops,
               const int min_accuracy,  float* solved) {
  // Convert examples to sets from vectors
  const std::unordered_set<Example> neg_set(neg_example.begin(), neg_example.end());
  const std::unordered_set<Example> pos_set(positive.begin(), positive.end());
  // Get a list of the names of all the feature holes in the conditional,
  // then store them in a vector because being able to access them in a
  // consistent order and by index is important for something we do later.
  const unordered_map<string, pair<Type, Dimension>> feature_hole_map =
      MapFeatureHoles(sketch);
  vector<string> feature_holes;
  for (const auto& p : feature_hole_map) {
    feature_holes.push_back(p.first);
  }
  const size_t feature_hole_count = feature_holes.size();
  
  if (FLAGS_debug) {
    cout << "Current Sketch: " << sketch << endl;
  }
  
  ast_ptr solution_cond = sketch;
  float current_best =  *solved;
  index_iterator c(ops.size(), feature_hole_count);
  bool keep_searching = true;
  int count = 0.0;
  #pragma omp parallel
  while (keep_searching) {
    // Use the indices given to us by the iterator to select ops for filling
    // our feature holes and create a model.
    vector<size_t> op_indicies;
    #pragma omp critical
    {
      if (c.has_next()) {
        op_indicies = c.next();
      } else {
        keep_searching = false;
        op_indicies = c.zeros();
      }
      count++;
    }
    ast_ptr filled = FillFeatureHoles(sketch, op_indicies, ops);
    if (filled != nullptr) {
      // TODO(jaholtz) Check if a solution exists
      #pragma omp critical
      {
        // Trying to return true for neg_set, and false otherwise
        const float score = PredicateL1(filled, neg_set, pos_set, false);
        if (keep_searching && score >= min_accuracy) {
          keep_searching = false;
          solution_cond = filled;
          current_best = score;
        } else if (score > current_best) {
          solution_cond = filled;
          current_best = score;
        }
      }
    }
  }
  *solved = current_best;
  return solution_cond;
}

void SynthesizeRule(const int &trap_size,
                    const vector<vector<vector<Example>>> &positive,
                    const vector<vector<Example>> &negative, const float& min_accuracy,
                    const vector<ast_ptr> sketches, const vector<ast_ptr> lib, 
                    const string& output_path) {

  vector<Example> positive_ex, negative_ex;
  vector<Example> trap_start = negative[trap_size + 1];
  // Starting with the transition just before the trap, we should never take the trap 
  // starting action.
  for (size_t i = 0; i < trap_size + 1; ++i) {
    const vector<Example> observation = negative[i];
    negative_ex.insert(negative_ex.end(), observation.begin(), observation.end());
  }

  // Any positive example with the trap starting action should be maintained.
  // These represent the safe states for taking that action.
  for (const auto& trajectory : positive) {
    for (const auto& observation : trajectory)  {
      for (const Example& example : observation) {
        if (example.result_ == trap_start[0].result_) {
          positive_ex.push_back(example);
        }
      }
    }
  }

  if (FLAGS_debug) {
    cout << "Negative Examples: " << negative_ex.size() << endl;
    cout << "Positive Examples: " << positive_ex.size() << endl;
  }

  // Consider each possible base sketch
  float current_best = 0.0;
  ast_ptr best_solution = nullptr;
  for (const auto &sketch : sketches) {
    float l2_score = 0.0;
    // check if any completion of the sketch is good.
    auto l2_solution = RuleL2(negative_ex, positive_ex, sketch, lib, min_accuracy,
                              &l2_score);
    if (l2_score > current_best) {
     best_solution = l2_solution;
     current_best = l2_score; 
    }
    if (FLAGS_debug) {
      cout << "Score: " << current_best << endl;
      cout << "Solution: " << best_solution << endl;
      cout << "- - - - -" << endl;
    }
  }
  // Write the solution out to a file.
  cout << "Score: " << current_best << endl;
  cout << "Banned Transition: " << trap_start[0].result_ << endl;
  cout << "Final Solution: " << best_solution << endl;
  ofstream output_file;
  output_file.open(output_path);
  const json output = best_solution->ToJson();
  output_file << std::setw(4) << output << std::endl;
  output_file.close();
  cout << endl;
}

void RulePIPS(const vector<vector<vector<Example>>>& positive,
              const vector<vector<vector<Example>>>& negative,
              const vector<ast_ptr> sketches, const vector<ast_ptr> lib,
              const float min_accuracy, const string& output_path) {
  // Synthesize one rule per negative example
  // TODO identify when negative example overlap
  for (size_t i = 0; i < negative.size(); ++i) {
    auto neg_example = negative[i];
    // Reversing it to check from the end.
    std::reverse(neg_example.begin(), neg_example.end());
    vector<ast_ptr> pruned_ops = PruneRuleOps(lib, neg_example);
    string output_name = output_path + "_" + std::to_string(i) + ".json";
    float current_best = 0.0;
    ast_ptr current_solution = nullptr;
    ast_ptr best_solution = nullptr;
    // First Synthesize a Trap to identify where things went wrong.
    for (const auto& sketch : sketches) {
      const float last_best = current_best;
      // TODO(jaholtz) is min trap ever a good idea?
      auto l2_solution = TrapL2(neg_example, positive, sketch, pruned_ops,
          100.0, &current_best); 
      if (current_best > last_best) {
        current_solution = l2_solution; 
      }
      if (FLAGS_debug) {
        cout << "Trap Size: " << current_best << endl;
        cout << "Trap Expression: " << current_solution << endl;
        cout << "- - - - -" << endl;
      }
    }
    if (FLAGS_debug) {
      cout << "Final Trap Size: " << current_best << endl;
      cout << "Final Trap Expression: " << current_solution << endl;
    }

    // Take the identified trap (score for now) and synthesize a rule
    // from it.
    SynthesizeRule(current_best, positive, neg_example, min_accuracy, sketches, lib, output_path);
  }
}

// TODO(currently writes to file, may want a call that doesn't do this).
void ldipsL3(const vector<Example>& demos,
      const vector<pair<string, string>>& transitions,
      const vector<ast_ptr> lib,
      const int sketch_depth,
      const float min_accuracy,
      const string& output_path) {

  vector<Example> examples = demos;
  // Enumerate possible sketches
  const auto sketches = EnumerateSketches(sketch_depth);

  // For each input/output pair
  for (const auto& transition : transitions) {
    // Skipping already synthesized conditions, allows for very basic
    // checkpointing.
    const string output_name =
      output_path + transition.first + "_" + transition.second + ".json";
    if (ExistsFile(output_name)) {
      continue;
    }
    cout << "----- " << transition.first << "->";
    cout << transition.second << " -----" << endl;
    float current_best = 0.0;
    ast_ptr current_solution = nullptr;
    for (const auto& sketch : sketches) {

      // Attempt L2 Synthesis with current sketch.
      current_solution = ldipsL2(sketch, examples, lib, transition,
          min_accuracy, current_solution, &current_best);
      if (current_best >= min_accuracy) break;
      if (FLAGS_debug) {
        cout << "Score: " << current_best << endl;
        cout << "Solution: " << current_solution << endl;
        cout << "- - - - -" << endl;
      }
    }
    // Write the solution out to a file.
    cout << "Score: " << current_best << endl;
    cout << "Final Solution: " << current_solution << endl;
    ofstream output_file;
    output_file.open(output_name);
    const json output = current_solution->ToJson();
    output_file << std::setw(4) << output << std::endl;
    output_file.close();

    // Filter out Examples used by this transition
    examples = FilterExamples(examples, transition);

    cout << endl;
  }
}

void DIPR(const vector<Example>& demos,
      const vector<ast_ptr>& programs,
      const vector<pair<string, string>>& transitions,
      const vector<ast_ptr> lib,
      const int sketch_depth,
      const float min_accuracy,
      const string& output_path) {

  vector<Example> examples = demos;
  // Enumerate possible sketches
  const auto sketches = EnumerateSketches(sketch_depth);

  // For each input/output pair
  for (size_t i = 0; i < programs.size(); ++i) {
    // TODO(jaholtz) make sure programs and transitions are in the same
    // order.
    const auto transition = transitions[i];
    ast_ptr best_program = programs[i];
    cout << "----- " << transition.first << "->";
    cout << transition.second << " -----" << endl;
    cout << "Initial Program:" << best_program << endl;

    // Checking Initial accuracy
    float pos = 0;
    float neg = 0;
    float best_score =
      ScorePredicate(best_program, transition, examples, &pos, &neg);

    cout << "Initial Score: " << best_score << endl;
    if (best_score >= min_accuracy) {
      cout << "Sufficient Performance, Skipping" << endl;
      cout << endl;
      ofstream output_file;
      const string output_name =
        output_path + transition.first + "_" + transition.second + ".json";
      output_file.open(output_name);
      const json output = best_program->ToJson();
      output_file << std::setw(4) << output << std::endl;
      output_file.close();
      continue;
    }

    // Search all possible extensions of the current program.
    // TODO(jaholtz) iterative over both sketches separately
    for (const auto& sketch : sketches) {
      // Extend the Sketch
      if (FLAGS_debug) {
        cout << "Pos: " << pos << " Neg: " << neg << endl;
      }
      ast_ptr candidate = ExtendPred(programs[i], sketch, sketch, pos, neg);

      // Attempt L2 Synthesis with the extended sketch.
      best_program = ldipsL2(candidate, examples, lib, transition,
          min_accuracy, best_program, &best_score);
      Z3_reset_memory();

      if (best_score >= min_accuracy) break;
    }
    // Write the solution out to a file.
    cout << "Final Solution: " << best_program << endl;
    cout << "Final Score: " << best_score << endl;
    ofstream output_file;
    const string output_name =
      output_path + transition.first + "_" + transition.second + ".json";
    output_file.open(output_name);
    const json output = best_program->ToJson();
    output_file << std::setw(4) << output << std::endl;
    output_file.close();

    // Filter out Examples used by this transition
    examples = FilterExamples(examples, transition);
    cout << endl;
  }
}

void SRTR(const vector<Example>& demos,
      const vector<ast_ptr>& programs,
      const vector<pair<string, string>>& transitions,
      const string& output_path) {
  // For each branch in the existing sketch
  for (size_t i = 0; i < programs.size(); ++i) {
    ast_ptr sketch = programs[i];
    const auto transition = transitions[i];

    // Checking Initial accuracy
    const SymEntry out(transition.second);
    const SymEntry in(transition.first);

    // Split up all the examples into a "yes" set or a "no" set based on
    // whether the result for the example matches the current example's
    // behavior.
    unordered_set<Example> yes;
    unordered_set<Example> no;
    SplitExamples(demos, transition, &yes, &no);
    if (yes.size() + no.size() > 0) {
      cout << "----- " << transition.first << "->";
      cout << transition.second << " -----" << endl;
      cout << "Initial Program:" << endl << sketch << endl;

      float current_best = CheckModelAccuracy(sketch, yes, no);
      cout << "Initial Score: " << current_best << endl;

      // Find best performing completion of current sketch
      sketch = Srtrize(sketch);
      const double score = PredicateL1(sketch, yes, no, true);

      cout << "Adjusted Program: " << endl << sketch << endl;
      cout << "Final Score: " << score << endl;
      cout << "- - - - - -" << endl;
      Z3_reset_memory();

      // Write the solution out to a file.
      ofstream output_file;
      const string output_name =
        output_path + transition.first + "_" + transition.second + ".json";
      output_file.open(output_name);
      const json output = sketch->ToJson();
      output_file << std::setw(4) << output << std::endl;
      output_file.close();

      // Filter out Examples used by this transition
      // examples = FilterExamples(examples, transition);
      cout << endl;
    }
  }
}

vector<ast_ptr> EnumerateCompletions(const ast_ptr sketch, const vector<ast_ptr>& expressions) {
  vector<ast_ptr> completions; 

  // Get a list of the names of all the feature holes in the conditional,
  // then store them in a vector because being able to access them in a
  // consistent order and by index is important for something we do later.
  const unordered_map<string, pair<Type, Dimension>> feature_hole_map =
      MapFeatureHoles(sketch);
  vector<string> feature_holes;
  for (const auto& p : feature_hole_map) {
    feature_holes.push_back(p.first);
  }
  const size_t feature_hole_count = feature_holes.size();
  
  ast_ptr solution_cond = sketch;
  index_iterator c(expressions.size(), feature_hole_count);
  while (c.has_next()) {
    // Use the indices given to us by the iterator to select ops for filling
    // our feature holes and create a model.
    vector<size_t> op_indicies = c.next();
    ast_ptr filled = FillFeatureHoles(sketch, op_indicies, expressions);
    completions.push_back(filled);
  }
  return completions;
}

}  // namespace AST
