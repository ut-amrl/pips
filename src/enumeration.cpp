#include "ast.hpp"
#include "enumeration.hpp"
#include "util/timer.h"
#include "visitors/interp_visitor.hpp"

#include <eigen3/Eigen/Core>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

using Eigen::Vector3i;
using std::cout;
using std::dynamic_pointer_cast;
using std::endl;
using std::invalid_argument;
using std::make_shared;
using std::vector;

namespace AST {

// ast_ptr Enumerate::Visit(AST* node) { return ast_ptr(node); }
// Calculate function signature given examples
// TODO(jaholtz) handle non-float signatures
vector<float> CalcSig(ast_ptr function, const vector<Example>& examples) {
  vector<float> signature;
  try {
    for (Example example : examples) {
      Interp interp(example);
      ast_ptr result = function->Accept(&interp);
      num_ptr result_cast = dynamic_pointer_cast<Num>(result);
      signature.push_back(result_cast->value_);
    }
  } catch (invalid_argument& e) {
    // Vector should be empty anyway, but just in case...
    signature.clear();
  }
  return signature;
}

CumulativeFunctionTimer sig_timer("CalcSigs");
vector<vector<float>> CalcSigs(const vector<ast_ptr>& functions,
                               const vector<Example>& examples) {
  CumulativeFunctionTimer::Invocation invoke(&sig_timer);
  vector<float>* updates = new vector<float>[functions.size()];
#pragma omp parallel for
  for (size_t i = 0; i < functions.size(); ++i) {
    vector<float> sig = CalcSig(functions.at(i), examples);
    updates[i] = sig;
  }
  // vector<vector<float>> signatures;
  vector<vector<float>> signatures(updates, updates + functions.size());
  delete[] updates;
  cout << "Length: Signatures: " << signatures.size() << endl;
  return signatures;
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

void PruneFunctions(const vector<vector<float>>& new_sigs,
                    vector<ast_ptr>* functions, vector<vector<float>>* sigs) {
  vector<ast_ptr> unique_functions;
  vector<vector<float>> unique_sigs = *sigs;

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
const static bool kSigPruning = true;
vector<ast_ptr> RecEnumerate(const vector<ast_ptr>& roots,
                             const vector<ast_ptr>& inputs,
                             const vector<Example>& examples,
                             const vector<FunctionEntry>& library, int depth,
                             vector<vector<float>>* signatures) {
  vector<ast_ptr> result = Enumerate(roots, inputs, library);
  if (kSigPruning) {
    const vector<vector<float>> new_sigs = CalcSigs(result, examples);
    PruneFunctions(new_sigs, &result, signatures);
  }
  if (depth > 1) {
    vector<ast_ptr> updated_inputs = inputs;
    updated_inputs.insert(updated_inputs.end(), result.begin(), result.end());
    const vector<ast_ptr> rec_result = RecEnumerate(
        result, updated_inputs, examples, library, --depth, signatures);
    result.insert(result.end(), rec_result.begin(), rec_result.end());
  }
  return result;
}

CumulativeFunctionTimer get_legal("GetLegalOperations");
// TODO(jaholtz) clean up this function, optimize
const static bool kNoDim = false;
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
    if (match_type && ((match_dim && match_index) || kNoDim)) {
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
              (in_dim == dimensions[in_index] || kNoDim)) {
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

} // namespace AST