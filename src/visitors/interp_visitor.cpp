#include "interp_visitor.hpp"

#include <eigen3/Eigen/Core> // Vector2f
#include <iostream>          // cout, endl
#include <memory>            // make_shared
#include <stdexcept>         // invalid_argument
#include <string>            // string

#include "../library_functions.hpp"

using Eigen::Vector2f;
using std::cout;
using std::endl;
using std::invalid_argument;
using std::make_shared;
using std::string;

namespace AST {

Interp::Interp() {}

Interp::Interp(const Example& world) : world_(world) {}

ast_ptr Interp::Visit(AST* node) { return ast_ptr(node); }

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

ast_ptr Interp::Visit(Num* node) { return make_shared<Num>(*node); }

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

ast_ptr Interp::Visit(Var* node) {
  if (world_.symbol_table_.find(node->name_) != world_.symbol_table_.end()) {
    if (node->type_ == NUM) {
      const float value = world_.symbol_table_[node->name_].GetValue();
      Num var_value(value, node->dims_);
      return make_shared<Num>(var_value);
    } else if (node->type_ == VEC) {
      const Vector2f float_vec = world_.symbol_table_[node->name_].GetValue();
      const Vector2f value = Vector2f(float_vec.data());
      Vec vec(value, node->dims_);
      return make_shared<Vec>(vec);
    } else {
      cout << "Error: Variable has unhandled type." << endl;
    }
  }
  cout << "Error: Variable not in symbol table" << endl;
  return make_shared<Var>(*node);
}

ast_ptr Interp::Visit(Vec* node) { return make_shared<Vec>(*node); }

} // namespace AST