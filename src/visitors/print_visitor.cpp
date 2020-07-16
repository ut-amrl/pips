#include "print_visitor.hpp"

#include <iostream>  // cout, endl
#include <memory>    // make_shared
#include <string>    // to_string

using std::cout;
using std::endl;
using std::make_shared;
using std::to_string;

namespace AST {

ast_ptr Print::Visit(AST* node) { return ast_ptr(node); }

ast_ptr Print::Visit(BinOp* node) {
  depth_++;
  program_ += node->op_ + "(";
  node->left_->Accept(this);
  program_ += ", ";
  node->right_->Accept(this);
  program_ += ")";
  return make_shared<BinOp>(*node);
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

ast_ptr Print::Visit(UnOp* node) {
  depth_++;
  program_ += node->op_ + "(";
  node->input_->Accept(this);
  program_ += ")";
  return make_shared<UnOp>(*node);
}

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

ast_ptr Print::Visit(Vec* node) {
  program_ += "<";
  program_ += to_string(node->value_.x());
  program_ += ", ";
  program_ += to_string(node->value_.y());
  program_ += ">";
  return make_shared<Vec>(*node);
}

void Print::Display() {
  cout << program_ << endl;
  program_ = "";
  depth_ = 0;
}

}  // namespace AST