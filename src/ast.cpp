#include "ast.hpp"
#include <iostream>

using Eigen::Vector3i;
using std::cout;
using std::dynamic_pointer_cast;
using std::endl;
using std::make_shared;
using std::string;
using std::to_string;

namespace AST {

// Constructors
AST::AST(Vector3i dims, Type type) : dims_(dims), type_(type) {}

AST::~AST(){};

Num::Num(const float& value, const Vector3i dims)
    : AST(dims, NUM), value_(value) {}

BinOp::BinOp(ast_ptr left, ast_ptr right, const string& op)
    : AST({0, 0, 0}, OP), left_(left), right_(right), op_(op) {}

// Necessary to get the automatic casting correct
ast_ptr Num::Accept(class Visitor* v) { return v->Visit(this); }

ast_ptr AST::Accept(class Visitor* v) { return v->Visit(this); }

ast_ptr BinOp::Accept(class Visitor* v) { return v->Visit(this); }
// End Casting Calls

// Print Visitor
ast_ptr Print::Visit(AST* node) { return ast_ptr(node); }

ast_ptr Print::Visit(Num* node) {
  program_ += to_string(node->value_);
  return std::make_shared<Num>(*node);
}

ast_ptr Print::Visit(BinOp* node) {
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
}

// Interp Visitor
ast_ptr Interp::Visit(AST* node) { return ast_ptr(node); }

ast_ptr Interp::Visit(Num* node) { return std::make_shared<Num>(*node); }

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

}  // namespace AST
