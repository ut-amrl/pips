#include "enumeration.hpp"

#include <gflags/gflags.h>
#include <z3++.h>

#include <eigen3/Eigen/Core>
#include <iostream>
#include <memory>
#include <ostream>
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
#include "visitors/deepcopy_visitor.hpp"
#include "visitors/fillhole_visitor.hpp"
#include "visitors/interp_visitor.hpp"
#include "visitors/print_visitor.hpp"
#include "visitors/tosmtlib_visitor.hpp"

using Eigen::Vector3i;
using namespace std;
using terminal_colors::ColorTerminal;
using terminal_colors::ResetTerminal;

bool dim_checking = true;
bool sig_pruning = true;
// DECLARE_bool(dim_checking);
// DECLARE_bool(sig_pruning);

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
vector<ast_ptr> EnumerateSketchesHelper(int depth) {
  vector<ast_ptr> sketches;

  if (depth == 0) {
    // Return [true, false]
    bool_ptr t = make_shared<Bool>(true);
    bool_ptr f = make_shared<Bool>(false);
    // sketches.push_back(t);
    // sketches.push_back(f);
    return sketches;
  }

  // TODO(jaholtz) make sure we can fill in any dimension of feature here.
  const string depth_string = std::to_string(depth);
  Feature f("fX" + depth_string, {0,0,0}, NUM);

  // Depth > 0
  vector<ast_ptr> rec_sketches = EnumerateSketchesHelper(depth - 1);

  if (depth == 1) {
    sketches.push_back(make_shared<Feature>(f));
    return sketches;
  }

  for (auto skt : rec_sketches) {
    if (skt->type_ != BOOL) {
      std::shared_ptr<BinOp> andf = make_shared<BinOp>(make_shared<Feature>(f), skt, "And");
      std::shared_ptr<BinOp> orf = make_shared<BinOp>(make_shared<Feature>(f), skt, "Or");
      sketches.push_back(andf);
      sketches.push_back(orf);
    }
  }
  return sketches;
}

vector<ast_ptr> EnumerateSketches(int depth) {
  vector<ast_ptr> sketches;
  int counter = 0;
  while (counter <= depth) {
    const vector<ast_ptr> current = EnumerateSketchesHelper(counter);
    sketches.insert(sketches.end(), current.begin(), current.end());
    counter++;
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
  vector<ast_ptr> result_list = roots;
  for (size_t i = 0; i < roots.size(); ++i) {
    ast_ptr node = roots.at(i);
    const vector<ast_ptr> new_nodes = GetLegalOps(node, inputs, library);
    CumulativeFunctionTimer::Invocation invoke(&update_list);
    // result_list.push_back(node);
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

  if (sig_pruning && examples.size() > 0) {
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

CumulativeFunctionTimer rec_enumerate_logistic("RecEnumerateLogistic");
vector<ast_ptr> RecEnumerateLogistic(const vector<ast_ptr>& roots,
                             const vector<ast_ptr>& inputs,
                             const vector<Example>& examples,
                             const vector<FunctionEntry>& library, int depth,
                             vector<Signature>* signatures) {
  CumulativeFunctionTimer::Invocation invoke(&rec_enumerate_logistic);
  for(ast_ptr each: roots){
    each->priority = 1;
  }
  vector<ast_ptr> feat = RecEnumerateHelper(roots, inputs, examples, library, depth, signatures);
  for(auto i = 0; i < feat.size(); i++){
    TernOp logistic(feat[i], make_shared<Num>(Num(0, {0, 0, 0})), make_shared<Num>(Num(0, {0, 0, 0})), "Logistic");
    BinOp flip(make_shared<TernOp>(logistic), make_shared<Bool>(true), "Flip");

    feat[i] = make_shared<BinOp>(flip);
  }
  return feat;
}

CumulativeFunctionTimer rec_enumerate("RecEnumerate");
vector<ast_ptr> RecEnumerate(const vector<ast_ptr>& roots,
                             const vector<ast_ptr>& inputs,
                             const vector<Example>& examples,
                             const vector<FunctionEntry>& library, int depth,
                             vector<Signature>* signatures) {
  CumulativeFunctionTimer::Invocation invoke(&rec_enumerate);
  for(ast_ptr each: roots){
    each->priority = 1;
  }
  return RecEnumerateHelper(roots, inputs, examples, library, depth, signatures);
}

ast_ptr FillFeatureHoles(ast_ptr sketch, const vector<size_t> &indicies,
                        const vector<ast_ptr> &ops) {
    Model m;

    // Get a list of the names of all the feature holes in the conditional,
    // then store them in a vector because being able to access them in a
    // consistent order and by index is important for something we do later.
    const unordered_map<string, pair<Type, Dimension>> feature_hole_map =
        MapFeatureHoles(sketch);

    vector<string> feature_holes;
    for (const auto &p : feature_hole_map) {
        feature_holes.push_back(p.first);
    }
    const size_t feature_hole_count = feature_holes.size();

    // For every feature hole...
    for (size_t i = 0; i < feature_hole_count; ++i) {
        // Get the name of a feature hole to fill and a possible value for it.
        const string &feature_hole = feature_holes[i];
        const pair<Type, Dimension> feature_hole_info =
            feature_hole_map.at(feature_hole);
        const Type feature_hole_type = feature_hole_info.first;
        const Dimension feature_hole_dims = feature_hole_info.second;
        const size_t index = indicies[i];
        const ast_ptr &op = ops[index];
        m[feature_hole] = op;
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
    filled->priority = FillHoles(filled, m);
    return filled;
}

void EnumerateL2(vector<ast_ptr>& full_sketches, vector<ast_ptr>& ops, ast_ptr sketch) {

    // Get a list of the names of all the feature holes in the conditional,
    // then store them in a vector because being able to access them in a
    // consistent order and by index is important for something we do later.
    const unordered_map<string, pair<Type, Dimension>> feature_hole_map =
        MapFeatureHoles(sketch);
    vector<string> feature_holes;
    for (const auto &p : feature_hole_map) {
        feature_holes.push_back(p.first);
    }
    const size_t feature_hole_count = feature_holes.size();

    // Start iterating through possible models. index_iterator is explained
    // separately.
    ast_ptr solution_cond = sketch;
    if (feature_hole_count > 0) {
        index_iterator c(ops.size(), feature_hole_count);

        vector<size_t> op_indicies;
        while (c.has_next()) {
            op_indicies = c.next();
            ast_ptr filled = FillFeatureHoles(sketch, op_indicies, ops);
            full_sketches.push_back(filled);
        }
    }
}

CumulativeFunctionTimer enumerate_l3("EnumerateL3");
vector<ast_ptr> EnumerateL3(vector<ast_ptr>& lib, int sketch_depth) {
    CumulativeFunctionTimer::Invocation invoke(&enumerate_l3);

    vector<ast_ptr> full_sketches;
    // Enumerate possible sketches
    const auto sketches = EnumerateSketches(sketch_depth);
    cout << "---- Number of Sketches Enumerated ----\n" << sketches.size() << endl;
    for (ast_ptr each : sketches) {
        cout << each << endl;
    }
    cout << endl << endl;

    for (const auto &sketch : sketches) {
        EnumerateL2(full_sketches, lib, sketch);
    }

    return full_sketches;
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
    if (match_type && ((match_dim && match_index) || !dim_checking)) {
      if (types.size() == 1) {
        // Generate signature and check before adding
        // Unary Op, create it and push back.
        // TODO(jaholtz) Currently using the "library" for determining
        // output dimensions and types. Should be able to use the operations
        // themselves to infer these without needing to enumerate them all.
        UnOp result = UnOp(node, func.op_, func.output_type_, func.output_dim_);
        result.priority = node->priority + 1;
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
              (in_dim == dimensions[in_index] || !dim_checking)) {
            // Use the correct order of inputs
            if (in_index == 0) {
              BinOp result = BinOp(input, node, func.op_, func.output_type_, func.output_dim_);
              result.priority = 1 + input->priority + node->priority;
              operations.push_back(make_shared<BinOp>(result));
            } else {
              BinOp result = BinOp(node, input, func.op_, func.output_type_, func.output_dim_);
              result.priority = 1 + input->priority + node->priority;
              operations.push_back(make_shared<BinOp>(result));
            }
          }
        }
      }
    }
  }
  return operations;
}

CumulativeFunctionTimer check_accuracy("CheckModelAccuracy");
double CheckModelAccuracy(const ast_ptr& cond,
                          const unordered_set<Example>& yes,
                          const unordered_set<Example>& no,
                          float* pos,
                          float* neg) {
  CumulativeFunctionTimer::Invocation invoke(&check_accuracy);
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
      // cout << "StartState: " << example.start_.GetString() << endl;
      // cout << "OutputState: " << example.result_.GetString() << endl;
      // cout << "Target Norm: " << example.symbol_table_.at("target").GetVector().x() << endl;
      // cout << "DoorState: " << example.symbol_table_.at("DoorState") << endl;
      // cout << "DoorPose: " << example.symbol_table_.at("DoorPose").GetVector().norm() << endl;
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
  CumulativeFunctionTimer::Invocation invoke(&check_accuracy);
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

void SplitExamples(const vector<Example>& examples,
    pair<string, string> transition,
    unordered_set<Example>* yes, unordered_set<Example>* no) {
  // Split up all the examples into a "yes" set or a "no" set based on
  // whether the result for the example matches the current example's
  // behavior.
  string out = transition.second;
  string in = transition.first;
  for (const Example& example : examples) {
    if (example.result_ == out && example.start_ == in) {
      yes->insert(example);
    } else if (example.start_ == in) {
      no->insert(example);
    }
  }
}

void SplitExamplesVector(const vector<Example>& examples,
    pair<string, string> transition,
    vector<Example>* yes, vector<Example>* no) {
  // Split up all the examples into a "yes" set or a "no" set based on
  // whether the result for the example matches the current example's
  // behavior.
  string out = transition.second;
  string in = transition.first;
  for (const Example& example : examples) {
    if (example.result_ == out && example.start_ == in) {
      yes->push_back(example);
    } else if (example.start_ == in) {
      no->push_back(example);
    }
  }
}

vector<Example> FilterExamples(const vector<Example>& examples,
    pair<string, string> transition) {
  vector<Example> copy;
  for (const Example& example : examples) {
    if(!(example.start_ == transition.first && example.result_ == transition.second)){
        copy.push_back(example);
    }
  }
  return copy;
}

vector<ast_ptr> RelativesOnly(const vector<ast_ptr>& ops) {
  vector<ast_ptr> output;
  for (ast_ptr ast : ops) {
    if (IsRelative(ast)) {
      output.push_back(ast);
    }
  }
  return output;
}

}  // namespace AST
