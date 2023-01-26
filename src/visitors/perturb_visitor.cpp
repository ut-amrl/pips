#include "perturb_visitor.hpp"

#include <eigen3/Eigen/Core>  // Vector2f
#include <iostream>           // cout, endl
#include <memory>             // make_shared
#include <stdexcept>          // invalid_argument
#include <string>             // string
#include <random>             // random

using Eigen::Vector2f;
using namespace std;

namespace AST {

vector<ast_ptr> findSimilar(const ast_ptr& base, vector<ast_ptr>& lib) {
    Perturb visitor(base, lib);
    base->Accept(&visitor);
    return visitor.sketches_;
}


Perturb::Perturb(const ast_ptr& base, vector<ast_ptr>& lib) : base_(base), lib_(lib) {}

ast_ptr Perturb::Visit(AST* node) { return ast_ptr(node); }

ast_ptr Perturb::Visit(TernOp* node){
    ast_ptr x = node->x_->Accept(this);
    ast_ptr a = node->a_->Accept(this);
    ast_ptr b = node->b_->Accept(this);
    const string op = node->op_;
    

    return make_shared<TernOp>(*node);
}

ast_ptr Perturb::Visit(BinOp* node) {
  ast_ptr left = node->left_->Accept(this);
  ast_ptr right = node->right_->Accept(this);
  const string op = node->op_;
  

  return make_shared<BinOp>(*node);
}

ast_ptr Perturb::Visit(UnOp* node) {
  ast_ptr input = node->input_->Accept(this);
  const string op = node->op_;
  
  return make_shared<UnOp>(*node);
}

ast_ptr Perturb::Visit(Bool* node) { return make_shared<Bool>(*node); }

ast_ptr Perturb::Visit(Feature* node) {
  if (node->current_value_ == nullptr) {
    throw invalid_argument("AST has unfilled feature holes");
  } else {
    ast_ptr result = node->current_value_->Accept(this);
    return result;
  }
}

ast_ptr Perturb::Visit(Num* node) { return make_shared<Num>(*node); }

ast_ptr Perturb::Visit(Param* node) {
  if (node->current_value_ == nullptr) {
    return make_shared<Param>(*node);
  } else {
    ast_ptr result = node->current_value_->Accept(this);
    return result;
  }
}

ast_ptr Perturb::Visit(Var* node) {
  return make_shared<Var>(*node);
}

ast_ptr Perturb::Visit(Vec* node) { return make_shared<Vec>(*node); }

}  // namespace AST
