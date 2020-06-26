#include "ast.hpp"

#include <eigen3/Eigen/src/Core/Matrix.h>

#include <exception>
#include <iostream>
#include <stdexcept>

#include "library_functions.hpp"
#include "util/timer.h"

using Eigen::Vector2f;
using Eigen::Vector3i;
using nlohmann::json;
using std::cout;
using std::dynamic_pointer_cast;
using std::endl;
using std::invalid_argument;
using std::make_shared;
using std::map;
using std::string;
using std::to_string;
using std::vector;

namespace AST {

ValueProxy::ValueProxy(SymEntry const* owner) : owner_(owner) {}

ValueProxy::operator float() const { return owner_->GetFloat(); }

ValueProxy::operator Vector2f() const { return owner_->GetVector(); }

SymEntry::SymEntry() : float_value_(0), vec_value_({0, 0}), is_num_(false) {}

SymEntry::SymEntry(const float value)
    : float_value_(value), vec_value_({0, 0}), is_num_(true) {}

SymEntry::SymEntry(const Eigen::Vector2f& value)
    : float_value_(0.0), vec_value_(value), is_num_(false) {}

ValueProxy SymEntry::GetValue() { return ValueProxy(this); }

const float SymEntry::GetFloat() const {
  if (!is_num_) {
    throw invalid_argument(
        "Attempted to get float "
        "value from non-float symbol.");
  }
  return float_value_;
}

const Vector2f SymEntry::GetVector() const {
  if (is_num_) {
    throw invalid_argument(
        "Attempted to get vector "
        "value from non-vector symbol.");
  }
  return vec_value_;
}

// Constructors
AST::AST(const Dimension& dims, const Type& type) : dims_(dims), type_(type) {}

AST::~AST(){};

Var::Var(const string& name, const Dimension& dims, const Type& type)
    : AST(dims, type), name_(name) {}

Param::Param(const string& name, const Dimension& dims, const Type& type)
    : AST(dims, type), name_(name) {}

Num::Num(const float& value, const Dimension& dims)
    : AST(dims, NUM), value_(value) {}

UnOp::UnOp(ast_ptr input, const string& op)
    : AST({0, 0, 0}, OP), input_(input), op_(op) {}

BinOp::BinOp(ast_ptr left, ast_ptr right, const string& op)
    : AST({0, 0, 0}, OP), left_(left), right_(right), op_(op) {}

BinOp::BinOp(ast_ptr left, ast_ptr right, const string& op, const Type& type,
             const Dimension& dim)
    : AST(dim, type), left_(left), right_(right), op_(op) {}

UnOp::UnOp(ast_ptr input, const string& op, const Type& type,
           const Dimension& dim)
    : AST(dim, type), input_(input), op_(op) {}

Vec::Vec(Vector2f value, Vector3i dims) : AST(dims, VEC), value_(value) {}

// Necessary to get the automatic casting correct
ast_ptr AST::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr BinOp::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Num::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Param::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr UnOp::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Var::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Vec::Accept(class Visitor* v) { return v->Visit(this); }
// End Casting Calls

// Print Visitor
ast_ptr Print::Visit(AST* node) { return ast_ptr(node); }

ast_ptr Print::Visit(Var* node) {
  program_ += node->name_;
  if (depth_ == 0) {
    program_ += " [" + to_string(node->dims_[0]) + ", " +
                to_string(node->dims_[1]) + ", " + to_string(node->dims_[2]) +
                "]";
  }
  depth_++;
  return make_shared<Var>(*node);
}

ast_ptr Print::Visit(Param* node) {
  program_ += node->name_;
  if (depth_ == 0) {
    program_ += " [" + to_string(node->dims_[0]) + ", " +
                to_string(node->dims_[1]) + ", " + to_string(node->dims_[2]) +
                "]";
  }
  depth_++;
  return make_shared<Param>(*node);
}

ast_ptr Print::Visit(Num* node) {
  program_ += to_string(node->value_);
  if (depth_ == 0) {
    program_ += " [" + to_string(node->dims_[0]) + ", " +
                to_string(node->dims_[1]) + ", " + to_string(node->dims_[2]) +
                "]";
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

ast_ptr Print::Visit(Vec* node) {
  program_ += "<";
  program_ += to_string(node->value_.x());
  program_ += ", ";
  program_ += to_string(node->value_.y());
  program_ += ">";
  return std::make_shared<Vec>(*node);
}

void Print::Display() {
  cout << program_ << endl;
  program_ = "";
  depth_ = 0;
}

// Interp Visitor
Interp::Interp() {}

Interp::Interp(const Example& world) : world_(world) {}

ast_ptr Interp::Visit(AST* node) { return ast_ptr(node); }

ast_ptr Interp::Visit(Num* node) { return std::make_shared<Num>(*node); }

ast_ptr Interp::Visit(Var* node) {
  if (world_.symbol_table_.find(node->name_) != world_.symbol_table_.end()) {
    if (node->type_ == NUM) {
      const float value = world_.symbol_table_[node->name_].GetValue();
      Num var_value(value, node->dims_);
      return std::make_shared<Num>(var_value);
    } else if (node->type_ == VEC) {
      const Vector2f float_vec = world_.symbol_table_[node->name_].GetValue();
      const Vector2f value = Vector2f(float_vec.data());
      Vec vec(value, node->dims_);
      return std::make_shared<Vec>(vec);
    } else {
      cout << "Error: Variable has unhandled type." << endl;
    }
  }
  cout << "Error: Variable not in symbol table" << endl;
  return make_shared<Var>(*node);
}

ast_ptr Interp::Visit(Param* node) {
  throw invalid_argument("AST has unfilled holes");
}

// TODO(jaholtz) Throw errors instead of printing
ast_ptr Interp::Visit(UnOp* node) {
  ast_ptr input = node->input_->Accept(this);
  const string op = node->op_;
  ast_ptr result = make_shared<UnOp>(*node);
  // One if clause per unary operation
  if (op == "Abs") {
    result = Abs(input);
  } else {
    throw invalid_argument("unknown unary operation `" + op + "'");
  }
  return result;
}

ast_ptr Interp::Visit(BinOp* node) {
  ast_ptr left = node->left_->Accept(this);
  ast_ptr right = node->right_->Accept(this);
  const string op = node->op_;
  ast_ptr result = make_shared<BinOp>(*node);
  // One if clause per binary operation
  if (op == "Plus") {
    result = Plus(left, right);
  } else {
    throw invalid_argument("unknown binary operation `" + op + "'");
  }
  return result;
}

ast_ptr Interp::Visit(Vec* node) { return std::make_shared<Vec>(*node); }

// ProblemGen visitor
ProblemGen::ProblemGen(vector<Example>& examples) : examples_(examples) {
  for (auto _ : examples_) {
    assertions_.push_back("");
  }
}

ast_ptr ProblemGen::Visit(AST* node) { return ast_ptr(node); }

ast_ptr ProblemGen::Visit(BinOp* node) {
  const string op = node->op_;
  string binop_smtlib;
  if (op == "Plus") {
    binop_smtlib += "(+ ";
  } else if (op == "Minus") {
    binop_smtlib += "(- ";
  } else if (op == "Times") {
    binop_smtlib += "(* ";
  } else if (op == "DividedBy") {
    binop_smtlib += "(/ ";
  } else if (op == "Pow") {
    binop_smtlib += "(^ ";
  } else {
    throw invalid_argument("unknown binary operation `" + op + "'");
  }
  for (string& assertion : assertions_) {
    assertion += binop_smtlib;
  }
  node->left_->Accept(this);
  for (string& assertion : assertions_) {
    assertion += " ";
  }
  node->right_->Accept(this);
  for (string& assertion : assertions_) {
    assertion += ")";
  }

  return make_shared<BinOp>(*node);
}

ast_ptr ProblemGen::Visit(Num* node) {
  const string num_string = to_string(node->value_);
  for (string& assertion : assertions_) {
    assertion += num_string;
  }
  return make_shared<Num>(*node);
}

ast_ptr ProblemGen::Visit(Param* node) {
  const string param_name = node->name_;
  for (string& assertion : assertions_) {
    assertion += param_name;
  }
  parameters_.insert(param_name);
  return make_shared<Param>(*node);
}

ast_ptr ProblemGen::Visit(UnOp* node) {
  const string op = node->op_;
  string unop_smtlib;
  if (op == "Abs") {
    unop_smtlib += "(abs ";
  } else if (op == "Cos") {
    unop_smtlib += "(cos ";
  } else if (op == "Sin") {
    unop_smtlib += "(sin ";
  } else {
    throw invalid_argument("unknown unary operation `" + op + "'");
  }
  for (string& assertion : assertions_) {
    assertion += unop_smtlib;
  }
  node->input_->Accept(this);
  for (string& assertion : assertions_) {
    assertion += ")";
  }

  return make_shared<UnOp>(*node);
}

ast_ptr ProblemGen::Visit(Var* node) {
  for (size_t i = 0; i < examples_.size(); ++i) {
    const string var_name = node->name_;
    // Can this be made const?
    Example& example = examples_[i];
    if (example.symbol_table_.find(var_name) != example.symbol_table_.end()) {
      string& assertion = assertions_[i];
      assertion += to_string(example.symbol_table_[var_name].GetFloat());
    } else {
      // TODO(simon) fail harder
      cout << "Variable not found in table!!!" << endl;
    }
  }
  return make_shared<Var>(*node);
}

ast_ptr ProblemGen::Visit(Vec* node) {
  throw invalid_argument("vectors are not yet supported");
}

string ProblemGen::Get() {
  string ret;

  for (const string& param_name : parameters_) {
    ret += "(declare-fun " + param_name + " () Real)\n";
  }

  for (size_t i = 0; i < assertions_.size(); ++i) {
    const string& assertion = assertions_[i];
    const Example& example = examples_[i];

    ret += "(assert (= ";
    ret += to_string(example.result_.GetFloat());
    ret += " ";
    ret += assertion;
    ret += "))\n";
  }

  assertions_.clear();
  examples_.clear();
  parameters_.clear();

  return ret;
}

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

}  // namespace AST
