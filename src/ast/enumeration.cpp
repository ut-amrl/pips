#include "enumeration.hpp"

#include <gflags/gflags.h>
#include <z3++.h>

#include <eigen3/Eigen/Core>
#include <iostream>
#include <memory>
#include <ostream>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <z3_api.h>

#include "../submodules/amrl_shared_lib/util/terminal_colors.h"
#include "../submodules/amrl_shared_lib/util/timer.h"
#include "ast.hpp"
#include "parsing.hpp"
#include "utils/nd_bool_array.hpp"
#include "visitors/deepcopy_visitor.hpp"
#include "visitors/fillhole_visitor.hpp"
#include "visitors/interp_visitor.hpp"
#include "visitors/print_visitor.hpp"
#include "visitors/tosmtlib_visitor.hpp"

using Eigen::Vector3i;
using std::cerr;
using std::cout;
using std::dynamic_pointer_cast;
using std::endl;
using std::invalid_argument;
using std::make_shared;
using std::ostream;
using std::pair;
using std::queue;
using std::string;
using std::to_string;
using std::unordered_map;
using std::unordered_set;
using std::vector;
using terminal_colors::ColorTerminal;
using terminal_colors::ResetTerminal;

DECLARE_bool(dim_checking);
DECLARE_bool(sig_pruning);

namespace AST {

ostream& operator<<(ostream& os, const FunctionEntry& fe) {
  os << fe.op_ << ":\t";
  for (size_t i = 0; i < fe.input_dims_.size(); ++i) {
    const string& type = TypeToString(fe.input_types_[i]);
    const Dimension& dims = fe.input_dims_[i];
    os << type << " [" << dims[0] << ", " << dims[1] << ", " << dims[2] << "]";
    if (i != fe.input_dims_.size() - 1) {
      os << ", ";
    }
  }
  os << " --> " << TypeToString(fe.output_type_) << " [" << fe.output_dim_[0]
     << ", " << fe.output_dim_[1] << ", " << fe.output_dim_[2] << "]";
  return os;
}

// ast_ptr Enumerate::Visit(AST* node) { return ast_ptr(node); }
// Calculate function signature given examples
Signature CalcSig(ast_ptr function, const vector<Example>& examples) {
  Signature sig;
  try {
    for (Example example : examples) {
      Interp interp(example);
      ast_ptr result = function->Accept(&interp);
      if (result->type_ == BOOL) {
        bool_ptr result_cast = dynamic_pointer_cast<Bool>(result);
        SymEntry result(result_cast->value_);
        sig.push_back(result);
      } else if (result->type_ == NUM) {
        num_ptr result_cast = dynamic_pointer_cast<Num>(result);
        SymEntry result(result_cast->value_);
        sig.push_back(result);
      } else if (result->type_ == VEC) {
        vec_ptr result_cast = dynamic_pointer_cast<Vec>(result);
        SymEntry result(result_cast->value_);
        sig.push_back(result);
      } else {
        throw invalid_argument("Unknown interpretation result type");
      }
    }
  } catch (invalid_argument& e) {
    // Vector should be empty anyway, but just in case...
    sig.clear();
  }
  return sig;
}

CumulativeFunctionTimer sig_timer("CalcSigs");
vector<Signature> CalcSigs(const vector<ast_ptr>& functions,
                           const vector<Example>& examples) {
  CumulativeFunctionTimer::Invocation invoke(&sig_timer);
  Signature* updates = new Signature[functions.size()];
#pragma omp parallel for
  for (size_t i = 0; i < functions.size(); ++i) {
    Signature sig = CalcSig(functions.at(i), examples);
    updates[i] = sig;
  }
  // vector<vector<float>> signatures;
  vector<Signature> signatures(updates, updates + functions.size());
  delete[] updates;
  return signatures;
}

// Grow a set of sketches to try and fill in
vector<ast_ptr> EnumerateSketches(int depth) {
  vector<ast_ptr> sketches;

  if (depth == 0) {
    // Return [true, false]
    bool_ptr t = make_shared<Bool>(true);
    bool_ptr f = make_shared<Bool>(false);
    sketches.push_back(t);
    sketches.push_back(f);
    return sketches;
  }

  // TODO(jaholtz) make sure we can fill in any dimension of feature here.
  const string depth_string = std::to_string(depth);
  Param p("pX" + depth_string, {0,0,0}, NUM);
  Feature f("fX" + depth_string, {0,0,0}, NUM);
  Param p1("pY" + depth_string, {0,0,0}, NUM);
  Feature f1("fY" + depth_string, {0,0,0}, NUM);
  std::shared_ptr<BinOp> great = make_shared<BinOp>(make_shared<Feature>(f),
                                 make_shared<Param>(p), "Gt");
  std::shared_ptr<BinOp> less = make_shared<BinOp>(make_shared<Feature>(f),
                                make_shared<Param>(p), "Lt");

  // Depth > 0
  vector<ast_ptr> rec_sketches = EnumerateSketches(depth - 1);

  if (depth == 1) {
    rec_sketches.push_back(great);
    rec_sketches.push_back(less);
    return rec_sketches;
  }

  //  Add the recursive sketches first (we want them to be earliest).
  for (auto skt : rec_sketches) {
    sketches.push_back(skt);
  }

  for (auto skt : rec_sketches) {
    if (skt->type_ != BOOL) {
      std::shared_ptr<BinOp> andg = make_shared<BinOp>(great, skt, "And");
      std::shared_ptr<BinOp> org = make_shared<BinOp>(great, skt, "Or");
      std::shared_ptr<BinOp> andl = make_shared<BinOp>(less, skt, "And");
      std::shared_ptr<BinOp> orl = make_shared<BinOp>(less, skt, "Or");
      sketches.push_back(andg);
      sketches.push_back(org);
      sketches.push_back(andl);
      sketches.push_back(orl);
    }
  }
  return sketches;
}

// Extends a predicate to match new examples, based on
// the performance of the existing examples, and the sketches.
ast_ptr ExtendPred(ast_ptr base, ast_ptr pos_sketch, ast_ptr neg_sketch,
    const float& pos, const float& neg) {
  // If we have both positive and negative examples
  // b'' && b || b'' && b'
  if (pos > 0 && neg > 0) {
    BinOp left(neg_sketch, base, "And");
    BinOp right(neg_sketch, pos_sketch, "And");
    BinOp result(make_shared<BinOp>(left), make_shared<BinOp>(right), "Or");
    return make_shared<BinOp>(result);
  }

  if (pos > 0) {
    BinOp result(base, pos_sketch, "Or");
    return make_shared<BinOp>(result);
  }

  BinOp result(base, neg_sketch, "And");
  return make_shared<BinOp>(result);
}

// Enumerate for a set of nodes
CumulativeFunctionTimer enum_timer("Enum");
CumulativeFunctionTimer update_list("UpdateList");
vector<ast_ptr> Enumerate(const vector<ast_ptr>& roots,
                          const vector<ast_ptr>& inputs,
                          const vector<FunctionEntry>& library) {
  CumulativeFunctionTimer::Invocation invoke(&enum_timer);
  vector<ast_ptr> result_list;
  for (size_t i = 0; i < roots.size(); ++i) {
    ast_ptr node = roots.at(i);
    const vector<ast_ptr> new_nodes = GetLegalOps(node, inputs, library);
    CumulativeFunctionTimer::Invocation invoke(&update_list);
    result_list.push_back(node);
    result_list.insert(result_list.end(), new_nodes.begin(), new_nodes.end());
  }
  return result_list;
}

void PruneFunctions(const vector<Signature>& new_sigs,
                    vector<ast_ptr>* functions, vector<Signature>* sigs) {
  vector<ast_ptr> unique_functions;
  vector<Signature> unique_sigs = *sigs;

  // For every signature in vector new_sigs..
  for (size_t i = 0; i < new_sigs.size(); ++i) {
    // If the signature vector is empty (indicating a function with a hole) or
    // signature is not in the old signature list...
    if (new_sigs[i].empty() ||
        std::find(sigs->begin(), sigs->end(), new_sigs[i]) == sigs->end()) {
      // Add the signature to the old signature list, and
      sigs->push_back(new_sigs[i]);
      // Add the corresponding function to the list of unique functions.
      unique_functions.push_back(functions->at(i));
    }
  }
  // Update the function list the user passed in to only have unique functions.
  *functions = unique_functions;
}

// Enumerate up to some depth for a set of nodes.
vector<ast_ptr> RecEnumerateHelper(const vector<ast_ptr>& roots,
                                   const vector<ast_ptr>& inputs,
                                   const vector<Example>& examples,
                                   const vector<FunctionEntry>& library,
                                   int depth, vector<Signature>* signatures) {
  vector<ast_ptr> result = Enumerate(roots, inputs, library);
  if (FLAGS_sig_pruning) {
    const vector<Signature> new_sigs = CalcSigs(result, examples);
    PruneFunctions(new_sigs, &result, signatures);
  }
  if (depth > 1) {
    vector<ast_ptr> updated_inputs = inputs;
    updated_inputs.insert(updated_inputs.end(), result.begin(), result.end());
    const vector<ast_ptr> rec_result = RecEnumerateHelper(
        result, updated_inputs, examples, library, --depth, signatures);
    result.insert(result.end(), rec_result.begin(), rec_result.end());
  }
  return result;
}

CumulativeFunctionTimer rec_enumerate("RecEnumerate");
vector<ast_ptr> RecEnumerate(const vector<ast_ptr>& roots,
                             const vector<ast_ptr>& inputs,
                             const vector<Example>& examples,
                             const vector<FunctionEntry>& library, int depth,
                             vector<Signature>* signatures) {
  CumulativeFunctionTimer::Invocation invoke(&rec_enumerate);
  return RecEnumerateHelper(roots, inputs, examples, library, depth,
                            signatures);
}

CumulativeFunctionTimer get_legal("GetLegalOperations");
// TODO(jaholtz) clean up this function, optimize
vector<ast_ptr> GetLegalOps(ast_ptr node, vector<ast_ptr> inputs,
                            const vector<FunctionEntry>& library) {
  CumulativeFunctionTimer::Invocation invoke(&get_legal);
  vector<ast_ptr> operations;
  const Type type = node->type_;
  const Vector3i dimension = node->dims_;

  // Branch based on all possible function applications
  for (const FunctionEntry& func : library) {
    // Get the required input types
    const vector<Type> types = func.input_types_;
    const vector<Vector3i> dimensions = func.input_dims_;
    int t_index = -1;
    int d_index = -1;
    // Does this function accept an entry with matching type and dimension
    const bool match_type = IndexInVector(types, type, &t_index);
    // TODO(jaholtz) find a solution for "matches any dimension/dimensionless"
    const bool match_dim = IndexInVector(dimensions, dimension, &d_index);
    const bool match_index = t_index == d_index;
    // We can create operations with this then.
    if (match_type && ((match_dim && match_index) || !FLAGS_dim_checking)) {
      if (types.size() == 1) {
        // Generate signature and check before adding
        // Unary Op, create it and push back.
        // TODO(jaholtz) Currently using the "library" for determining
        // output dimensions and types. Should be able to use the operations
        // themselves to infer these without needing to enumerate them all.
        UnOp result = UnOp(node, func.op_, func.output_type_, func.output_dim_);
        operations.push_back(make_shared<UnOp>(result));
      } else {
        // Binary Op, have to find some other argument.
        // Identify which index we need to be looking at.
        int in_index = (t_index == 0) ? 1 : 0;
        for (auto input : inputs) {
          Type in_type = input->type_;
          Vector3i in_dim = input->dims_;
          // If matches the function signature
          if (in_type == types[in_index] &&
              (in_dim == dimensions[in_index] || !FLAGS_dim_checking)) {
            // Use the correct order of inputs
            if (in_index == 0) {
              BinOp result = BinOp(input, node, func.op_, func.output_type_,
                                   func.output_dim_);
              operations.push_back(make_shared<BinOp>(result));
            } else {
              BinOp result = BinOp(node, input, func.op_, func.output_type_,
                                   func.output_dim_);
              operations.push_back(make_shared<BinOp>(result));
            }
          }
        }
      }
    }
  }
  return operations;
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
      throw invalid_argument("All model values must be numeral");
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

double CheckModelAccuracy(const ast_ptr& cond,
                          const unordered_set<Example>& yes,
                          const unordered_set<Example>& no,
                          float* pos,
                          float* neg) {
  // Create a variable for keeping track of the number of examples where we get
  // the expected result.
  size_t satisfied = 0;
  // Count for how many "yes" examples the interpretation of the condition is
  // true.
  for (const Example& example : yes) {
    const ast_ptr result = Interpret(cond, example);
    bool_ptr result_cast = dynamic_pointer_cast<Bool>(result);
    if (result_cast->value_) {
      satisfied += 1;
    } else {
      *pos += 1;
    }
  }

  // Count for how many "no" examples the interpretation of the condition is
  // false.
  for (const Example& example : no) {
    const ast_ptr result = Interpret(cond, example);
    bool_ptr result_cast = dynamic_pointer_cast<Bool>(result);
    if (!result_cast->value_) {
      satisfied += 1;
    } else {
      *neg += 1;
    }
  }

  // Compute the final percentage of satisfied examples to all examples.
  const double sat_ratio = (double)satisfied / (yes.size() + no.size());
  return sat_ratio;
}

double CheckModelAccuracy(const ast_ptr& cond,
                          const unordered_set<Example>& yes,
                          const unordered_set<Example>& no) {
  // Create a variable for keeping track of the number of examples where we get
  // the expected result.
  size_t satisfied = 0;
  // Count for how many "yes" examples the interpretation of the condition is
  // true.
  for (const Example& example : yes) {
    const ast_ptr result = Interpret(cond, example);
    bool_ptr result_cast = dynamic_pointer_cast<Bool>(result);
    if (result_cast->value_) satisfied += 1;
  }

  // Count for how many "no" examples the interpretation of the condition is
  // false.
  for (const Example& example : no) {
    const ast_ptr result = Interpret(cond, example);
    bool_ptr result_cast = dynamic_pointer_cast<Bool>(result);
    if (!result_cast->value_) {
      satisfied += 1;
    }
  }

  // Compute the final percentage of satisfied examples to all examples.
  const double sat_ratio = (double)satisfied / (yes.size() + no.size());
  return sat_ratio;
}

CumulativeFunctionTimer solve_smtlib("SolveSMTLIBProblem");
Model SolveSMTLIBProblem(const string& problem) {
  // CumulativeFunctionTimer::Invocation invoke(&solve_smtlib);
  z3::context context;
  z3::optimize solver(context);
  solver.from_string(problem.c_str());
  z3::check_result result = solver.check();
  if (result == z3::sat) {
    z3::model m = solver.get_model();
    return Z3ModelToMap(m);
  } else {
    throw invalid_argument("UNSAT");
  }
}

CumulativeFunctionTimer make_smtlib("MakeSMTLIBProblem");
string MakeSMTLIBProblem(const unordered_set<Example>& yes,
                         const unordered_set<Example>& no,
                         const ast_ptr program) {
  CumulativeFunctionTimer::Invocation invoke(&make_smtlib);

  // Create empty sets to hold data needed for program generation. The params
  // set will track the names of parameter holes discovered, and the assertions
  // set will hold the assertions we generate.
  unordered_set<string> params;
  unordered_set<string> assertions;

  // Make a new set that includes all examples.
  unordered_set<Example> all_examples(yes);
  all_examples.insert(no.cbegin(), no.cend());
  // For every example, ...
  for (const Example& example : all_examples) {
    // Partially evaluate the program with the examples as much as possible.
    ast_ptr partial = Interpret(program, example);
    // Create an SMT-LIB expression from the program and that example.
    ToSMTLIB converter = AstToSMTLIB(partial, example);
    const string smtlib = converter.Get();
    // Add full assertions using that expression to our assertion list. It's
    // important to check whether the example is in both sets because we could
    // have contradictory examples.
    if (yes.find(example) != yes.cend())
      assertions.insert("(assert-soft " + smtlib + ")");
    if (no.find(example) != no.cend())
      assertions.insert("(assert-soft (not " + smtlib + "))");

    // If there are any parameter holes, make note of them too.
    const unordered_set<string> example_params = converter.GetParams();
    params.insert(example_params.cbegin(), example_params.cend());
  }

  // Create a string where our full SMT-LIB problem will be created.
  //
  // Any SMT-LIB formula including division by 0 is SAT, so create a new
  // function for "safe" division.
  //
  // TODO(simon) Find a smarter solution, since this could still possibly
  // result in SATs when it shouldn't.
  string problem = R"(
    (define-fun safe-div ((x!1 Real) (x!2 Real)) Real
      (ite (= x!2 0.0)
        0.0
        (/ x!1 x!2)))

    (define-fun cross ((x!1 Real) (y!1 Real) (x!2 Real) (y!2 Real)) Real
      (+ (* x!1 y!2) (* y!1 x!2)))

    (define-fun dot ((x!1 Real) (y!1 Real) (x!2 Real) (y!2 Real)) Real
      (+ (* x!1 x!2) (* y!1 y!2)))

    (define-fun sq ((x!1 Real)) Real
      (* x!1 x!1))

    (define-fun euc-dist-sq ((x!1 Real) (y!1 Real) (x!2 Real) (y!2 Real)) Real
      (+ (sq (- x!2 x!1)) (sq (- y!2 y!1))))

    (define-fun norm-sq ((x!1 Real) (y!1 Real)) Real
      (+ (sq x!1) (sq y!1)))

    (define-fun vec-x ((x!1 Real) (y!1 Real)) Real
      x!1)

    (define-fun vec-y ((x!1 Real) (y!1 Real)) Real
      y!1)
  )";

  problem = "";

  // For every parameter hole discovered, add a real constant to the problem.
  for (const string& param : params) {
    problem += "(declare-const " + param + " Real)\n";
  }

  // Add every assertion created previously to the problem.
  for (const string& assertion : assertions) {
    problem += assertion + "\n";
  }

  for (const string& param : params) {
    // minimize the absolute value of these constants. Closer to 0 is better
    // in the SRTR case, and ignoreable the rest of the time (because of
    // lexicographic optimization)
    const string absolute =
        "(ite (> 0 " + param + ") (- 0 " + param + ") " + param + ")";
    problem += "(minimize " + absolute + ")\n" ;
  }
  // cout << problem << endl;

  // Now that the problem string has been completely generated, return it.
  // There is no need to add instructions to the solver like (check-sat) or
  // (get-model) since those actions will be performed by us using the Z3 C++
  // API later.
  return problem;
}

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
      throw invalid_argument("no ops!");
    }
    if (holes_to_fill_ == 0) {
      throw invalid_argument("no holes to fill!");
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
  queue<vector<size_t>> indicies_;
  nd_bool_array visited_;
};

CumulativeFunctionTimer solve_conditional("SolveConditional");
vector<ast_ptr> SolveConditional(
    const vector<Example>& examples, const vector<ast_ptr>& ops,
    const vector<std::pair<ast_ptr, SymEntry>>& sketch, double min_accuracy) {
  CumulativeFunctionTimer::Invocation invoke(&solve_conditional);

  // Create a vector for putting solved conditions in. The order of conditions
  // in this vector will match the order that condition sketches appear in the
  // sketch parameter: ret[i] corresponds to sketch[i] for all i.
  vector<ast_ptr> ret;

  // For every pair of a sketch and an output...
  for (auto& cond_out : sketch) {
    // Split the condition/output pair into two variables for easier access.
    const ast_ptr& cond = cond_out.first;
    const SymEntry& out = cond_out.second;

    // Get a list of the names of all the feature holes in the conditional,
    // then store them in a vector because being able to access them in a
    // consistent order and by index is important for something we do later.
    const unordered_map<string, pair<Type, Dimension>> feature_hole_map =
        MapFeatureHoles(cond);
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
    for (const Example& example : examples) {
      if (example.result_ == out) {
        yes.insert(example);
      } else {
        no.insert(example);
      }
    }

    // Start iterating through possible models. index_iterator is explained
    // seperately.
    index_iterator c(ops.size(), feature_hole_count);
    ast_ptr solution_cond = nullptr;
    bool keep_searching = true;
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
      }

      Model m;

      // For every feature hole...
      for (size_t i = 0; i < feature_hole_count; ++i) {
        // Get the name of a feature hole to fill and a possible value for it.
        const string& feature_hole = feature_holes[i];
        const pair<Type, Dimension> feature_hole_info =
            feature_hole_map.at(feature_hole);
        const Type feature_hole_type = feature_hole_info.first;
        const Dimension feature_hole_dims = feature_hole_info.second;
        const size_t index = op_indicies[i];
        const ast_ptr& op = ops[index];

        // If the type of the selected op doesn't match, we can stop creating
        // this model.
        // TODO(jaholtz) We want the option to have generic dimension holes
        // to fill.
        if (op->type_ != feature_hole_type || op->dims_ != feature_hole_dims) {
          break;
        }

        // Since the type does match, make a copy of the op and put that copy
        // in the model.
        else {
          const ast_ptr op_copy = DeepCopyAST(op);
          m[feature_hole] = op_copy;
        }
      }

      // If after creating the model the number of filled holes is not the same
      // as the number of holes that ought to be filled (indicating a type
      // error), we can give up and try the next model.
      if (m.size() != feature_hole_count) {
        continue;
      }

      // Since no errors occured while creating the model, we can take the hole
      // values from it and use them to fill the holes in a copy of the
      // condition.
      ast_ptr cond_copy = DeepCopyAST(cond);
      FillHoles(cond_copy, m);

      // Now that we've built our candidate condition, build an SMT-LIB problem
      // and pass it to Z3 to attempt to solve.
      const string problem = MakeSMTLIBProblem(yes, no, cond_copy);
      Model solution;
      try {
        solution = SolveSMTLIBProblem(problem);
        FillHoles(cond_copy, solution);
        const double sat_ratio = CheckModelAccuracy(cond_copy, yes, no);
#pragma omp critical
        {
          if (keep_searching && sat_ratio >= min_accuracy) {
            keep_searching = false;
            solution_cond = cond_copy;
          }
        }
      } catch (const invalid_argument&) {
        // not SAT, do nothing
      }
    }
    if (solution_cond != nullptr) {
      ret.push_back(solution_cond);
    } else {
      throw invalid_argument("cannot solve sketch with given parameters");
    }
  }

  return ret;
}


CumulativeFunctionTimer solve_predicate("SolvePredicate");
ast_ptr SolvePredicate(
    const vector<Example>& examples, const vector<ast_ptr>& ops,
    const ast_ptr& sketch, const pair<string,string>& transition,
    double min_accuracy, float* solved) {
  CumulativeFunctionTimer::Invocation invoke(&solve_predicate);

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
  for (const Example& example : examples) {
    if (example.result_ == out && example.start_ == in) {
      yes.insert(example);
    } else if (example.start_ == in) {
      no.insert(example);
    }
  }

  // Start iterating through possible models. index_iterator is explained
  // seperately.
  ast_ptr solution_cond = sketch;
  float current_best =  0.0;
  if (feature_hole_count > 0) {
    index_iterator c(ops.size(), feature_hole_count);
    solution_cond = nullptr;
    bool keep_searching = true;
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
      }

      Model m;

      // For every feature hole...
      bool no_solve = false;
      for (size_t i = 0; i < feature_hole_count; ++i) {
        // Get the name of a feature hole to fill and a possible value for it.
        const string& feature_hole = feature_holes[i];
        const pair<Type, Dimension> feature_hole_info =
          feature_hole_map.at(feature_hole);
        const Type feature_hole_type = feature_hole_info.first;
        const Dimension feature_hole_dims = feature_hole_info.second;
        const size_t index = op_indicies[i];
        const ast_ptr& op = ops[index];
        // If the type of the selected op doesn't match, we can stop creating
        // this model.
        if (op->type_ != feature_hole_type) {
          break;
        }

        // Since the type does match, make a copy of the op and put that copy
        // in the model.
        else {
          // const ast_ptr op_copy = DeepCopyAST(op);
          m[feature_hole] = op;
        }
      }

      // If after creating the model the number of filled holes is not the same
      // as the number of holes that ought to be filled (indicating a type
      // error), we can give up and try the next model.
      if (m.size() != feature_hole_count) {
        no_solve = true;
      }

      // Since no errors occured while creating the model, we can take the hole
      // values from it and use them to fill the holes in a copy of the
      // condition.
      ast_ptr cond_copy = DeepCopyAST(sketch);
      FillHoles(cond_copy, m);

      if (!no_solve) {
        const string problem = MakeSMTLIBProblem(yes, no, cond_copy);
        // cout << "Current Problem" << endl;
        // cout << problem << endl;
        // cout << endl;
        Model solution;
        try {
          solution = SolveSMTLIBProblem(problem);
          FillHoles(cond_copy, solution);
          const double sat_ratio = CheckModelAccuracy(cond_copy, yes, no);
#pragma omp critical
          {
            if (keep_searching && sat_ratio >= min_accuracy) {
              keep_searching = false;
              solution_cond = cond_copy;
              current_best = sat_ratio;
            } else if (sat_ratio >= current_best) {
              solution_cond = cond_copy;
              current_best = sat_ratio;
            }
          }
        } catch (const invalid_argument&) {
          // not SAT, do nothing
        }
      }
    }
  } else {
    // Since no errors occured while creating the model, we can take the hole
    // values from it and use them to fill the holes in a copy of the
    // condition.
    ast_ptr cond_copy = DeepCopyAST(sketch);

    // Now that we've built our candidate condition, build an SMT-LIB problem
    // and pass it to Z3 to attempt to solve.
    const string problem = MakeSMTLIBProblem(yes, no, cond_copy);
    Model solution;
    try {
      solution = SolveSMTLIBProblem(problem);
      FillHoles(cond_copy, solution);
      const double sat_ratio = CheckModelAccuracy(cond_copy, yes, no);
      current_best = sat_ratio;
    } catch (const invalid_argument&) {
      solution_cond = nullptr;
    }
  }
  if (solution_cond != nullptr) {
    *solved = current_best;
  }
  return solution_cond;
}

}  // namespace AST
