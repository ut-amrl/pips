#include "ast.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include "util/timer.h"

using Eigen::Vector3i;
using std::cout;
using std::dynamic_pointer_cast;
using std::endl;
using std::make_shared;
using std::map;
using std::string;
using std::to_string;
using std::vector;

namespace AST {

// Constructors
AST::AST(Vector3i dims, Type type) : dims_(dims), type_(type) {}

AST::~AST(){};

Num::Num(const float& value, const Vector3i dims)
    : AST(dims, NUM), value_(value) {}

BinOp::BinOp(ast_ptr left, ast_ptr right, const string& op)
    : AST({0, 0, 0}, OP), left_(left), right_(right), op_(op) {}

UnOp::UnOp(ast_ptr input, const string& op)
    : AST({0, 0, 0}, OP), input_(input), op_(op) {}

BinOp::BinOp(ast_ptr left, ast_ptr right, const string& op, const Type& type,
             const Dimension& dim)
    : AST(dim, type), left_(left), right_(right), op_(op) {}

UnOp::UnOp(ast_ptr input, const string& op, const Type& type,
           const Dimension& dim)
    : AST(dim, type), input_(input), op_(op) {}

// Necessary to get the visitor pattern working
ast_ptr Num::Accept(class Visitor* v) { return v->Visit(this); }

ast_ptr AST::Accept(class Visitor* v) { return v->Visit(this); }

ast_ptr BinOp::Accept(class Visitor* v) { return v->Visit(this); }

ast_ptr UnOp::Accept(class Visitor* v) { return v->Visit(this); }
// End Casting Calls

// Print Visitor
ast_ptr Print::Visit(AST* node) { return ast_ptr(node); }

ast_ptr Print::Visit(Num* node) {
  program_ += to_string(node->value_);
  if (depth_ == 0) {
    program_ += " [" + to_string(node->dims_[0]) + ", " +
                to_string(node->dims_[1]) + ", " + to_string(node->dims_[2]) +
                " ]";
  }
  depth_++;
  return make_shared<Num>(*node);
}

ast_ptr Print::Visit(UnOp* node) {
  depth_++;
  program_ += node->op_ + "(";
  node->input_->Accept(this);
  program_ += ")";
  return std::make_shared<UnOp>(*node);
}

ast_ptr Print::Visit(BinOp* node) {
  depth_++;
  program_ += node->op_ + "(";
  node->left_->Accept(this);
  program_ += ", ";
  node->right_->Accept(this);
  program_ += ")";
  return std::make_shared<BinOp>(*node);
}

void Print::Display() {
  cout << program_ << endl;
  program_ = "";
  depth_ = 0;
}

// TODO(jaholtz) Throw mistyped and misdimensioned exceptions
ast_ptr Plus(ast_ptr left, ast_ptr right) {
  const Vector3i dim_left = left->dims_;
  const Vector3i dim_right = right->dims_;
  const Type type_left = left->type_;
  const Type type_right = right->type_;
  // Type and Dimension Checking
  if (dim_left != dim_right) {
    cout << "ERROR Dimension Mismatch" << endl;
  }
  if (type_left != type_right) {
    cout << "ERROR Type Mismatch" << endl;
  }
  // Casting to the correct type any type you can add should have a case.
  if (type_left == NUM) {
    num_ptr left_cast = dynamic_pointer_cast<Num>(left);
    num_ptr right_cast = dynamic_pointer_cast<Num>(right);
    Num result(left_cast->value_ + right_cast->value_, dim_left);
    return make_shared<Num>(result);
  } else {
    cout << "Error: Illegal Types passed to Plus" << endl;
  }
  return ast_ptr(left);
}

ast_ptr Absolute(ast_ptr input) {
  const Type type_input = input->type_;
  // Casting to the correct type any type you can add should have a case.
  if (type_input == NUM) {
    num_ptr input_cast = dynamic_pointer_cast<Num>(input);
    Num result(fabs(input_cast->value_), input->dims_);
    return make_shared<Num>(result);
  } else {
    cout << "Error: Illegal Types passed to Absolute" << endl;
  }
  return ast_ptr(input);
}

// Interp Visitor
Interp::Interp() {}

Interp::Interp(const Example& world) : world_(world) {}

ast_ptr Interp::Visit(AST* node) { return ast_ptr(node); }

ast_ptr Interp::Visit(Num* node) { return std::make_shared<Num>(*node); }

// TODO(jaholtz) Throw errors instead of printing
ast_ptr Interp::Visit(UnOp* node) {
  ast_ptr input = node->input_->Accept(this);
  const string op = node->op_;
  ast_ptr result = make_shared<UnOp>(*node);
  // One if clause per unary operation
  if (op == "Absolute") {
    result = Absolute(input);
  } else {
    cout << "ERROR: Unhandled Operation" << endl;
  }
  return result;
}

// TODO(jaholtz) Throw errors instead of printing
ast_ptr Interp::Visit(BinOp* node) {
  ast_ptr left = node->left_->Accept(this);
  ast_ptr right = node->right_->Accept(this);
  const string op = node->op_;
  ast_ptr result = make_shared<BinOp>(*node);
  // One if clause per binary operation
  if (op == "Plus") {
    result = Plus(left, right);
  } else {
    cout << "ERROR: Unhandled Operation" << endl;
  }
  return result;
}

// Calculate function signature given examples
// TODO(jaholtz) handle non-float signatures
vector<float> CalcSig(ast_ptr function, const vector<Example>& examples) {
  vector<float> signature;
  for (Example example : examples) {
    Interp interp(example);
    ast_ptr result = function->Accept(&interp);
    num_ptr result_cast = dynamic_pointer_cast<Num>(result);
    signature.push_back(result_cast->value_);
  }
  return signature;
}

CumulativeFunctionTimer sig_timer("CalcSigs");
// TODO(jaholtz) Check function signature equality
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
    result_list.insert(result_list.end(), new_nodes.begin(), new_nodes.end());
  }
  return result_list;
}

void PruneFunctions(const vector<vector<float>>& new_sigs,
                    vector<ast_ptr>* functions, vector<vector<float>>* sigs) {
  vector<ast_ptr> unique_functions;
  vector<vector<float>> unique_sigs = *sigs;
  for (auto i = 0; i < new_sigs.size(); ++i) {
    if (std::find(sigs->begin(), sigs->end(), new_sigs[i]) == sigs->end()) {
      sigs->push_back(new_sigs[i]);
      unique_functions.push_back(functions->at(i));
    }
  }
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

}  // namespace AST
