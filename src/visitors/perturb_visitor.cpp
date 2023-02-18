#include "perturb_visitor.hpp"
#include "deepcopy_visitor.hpp"
#include "print_visitor.hpp"

#include <eigen3/Eigen/Core>  // Vector2f
#include <iostream>           // cout, endl
#include <memory>             // make_shared
#include <stdexcept>          // invalid_argument
#include <string>             // string
#include <random>             // random

using Eigen::Vector2f;
using namespace std;

namespace AST {

// TODO (optional): add axioms/observational equivalence
// TODO (optional): replace last iteration's parameters with 0s while enumerating for sketch equivalence checking
// TODO (optional): start optimization from the last sketch's parameters (this should increase the convergence speed)

vector<ast_ptr> findSimilar(ast_ptr& base, vector<ast_ptr>& lib, vector<ast_ptr>& sketches) {
    // We randomly perturb the base program in 4 ways: see 1), 2), 3), and 4)
    ast_ptr copy = DeepCopyAST(base);

    Perturb visitor(copy, lib);

    // 1. Randomly add clauses with a base feature
    for(ast_ptr feat: sketches) {
        shared_ptr<BinOp> andf = make_shared<BinOp>(feat, copy, "And");
        shared_ptr<BinOp> orf = make_shared<BinOp>(feat, copy, "Or");

        visitor.add_candidate(andf);
        visitor.add_candidate(orf);
    }

    copy->Accept(&visitor);
    return visitor.sketches_;
}

void Perturb::add_candidate(ast_ptr prog) { 
    ast_ptr copy = DeepCopyAST(prog);
    string s = GetString(copy);
    if(hashes_.count(s) == 0){
        hashes_.insert(s);
        sketches_.push_back(copy);
    }
}

Perturb::Perturb(ast_ptr& base, vector<ast_ptr>& lib) : base_(base), lib_(lib) {}

ast_ptr Perturb::Visit(AST* node) { return ast_ptr(node); }

ast_ptr Perturb::Visit(TernOp* node){
    ast_ptr x = node->x_->Accept(this);
    ast_ptr a = node->a_->Accept(this);
    ast_ptr b = node->b_->Accept(this);
    const string op = node->op_;

    if(op == "Logistic") {
        // 4. Randomly perturb features by replacing subtrees
        ast_ptr child = node->x_;
        for(ast_ptr each: lib_) {
            if(each->dims_ == child->dims_) {
                node->x_ = each;
                base_->complexity_ = base_->complexity_ - child->complexity_ + each->complexity_;
                add_candidate(base_);
                node->x_ = child;
                base_->complexity_ = base_->complexity_ + child->complexity_ - each->complexity_;
            }
        }
    } else {
        // TODO (optional): Add support for any other ternaries here
    }

    return make_shared<TernOp>(*node);
}

ast_ptr Perturb::Visit(BinOp* node) {
    ast_ptr left = node->left_->Accept(this);
    ast_ptr right = node->right_->Accept(this);
    const string op = node->op_;

    // 2. Randomly switch ANDs and ORs
    if(op == "And") {
        node->op_ = "Or";
        add_candidate(base_);
        node->op_ = "And";
    }
    if(op == "Or") {
        node->op_ = "And";
        add_candidate(base_);
        node->op_ = "Or";
    }

    // 4. Randomly perturb features by replacing subtrees
    ast_ptr c_left = node->left_;
    ast_ptr c_right = node->right_;
    for(ast_ptr each: lib_) {
        if(each->dims_ == c_left->dims_) {
            node->left_ = each;
            base_->complexity_ = base_->complexity_ - left->complexity_ + each->complexity_;
            add_candidate(base_);
            node->left_ = c_left;
            base_->complexity_ = base_->complexity_ + left->complexity_ - each->complexity_;
        }

        if(each->dims_ == c_right->dims_) {
            node->right_ = each;
            base_->complexity_ = base_->complexity_ - right->complexity_ + each->complexity_;
            add_candidate(base_);
            node->right_ = c_right;
            base_->complexity_ = base_->complexity_ + right->complexity_ - each->complexity_;
        }
    }

    // 3. Randomly remove clauses
    if(op == "And" || op == "Or") {
        ast_ptr temp = node->right_; // right node

        node->right_ = right; // either a null op or removes a clause
        base_->complexity_ = base_->complexity_ - temp->complexity_ + right->complexity_;
        add_candidate(base_);
        node->right_ = temp; // reset
        base_->complexity_ = base_->complexity_ + temp->complexity_ - right->complexity_;

        // Try adding only the left and right subtree
        add_candidate(node->left_);
        add_candidate(node->right_);

        return node->right_;
    }

    return make_shared<BinOp>(*node);
}

ast_ptr Perturb::Visit(UnOp* node) {
    ast_ptr input = node->input_->Accept(this);
    const string op = node->op_;

    // 4. Randomly perturb features by replacing subtrees
    ast_ptr in = node->input_;
    for(ast_ptr each: lib_) {
        if(each->dims_ == in->dims_) {
            node->input_ = each;
            base_->complexity_ = base_->complexity_ - in->complexity_ + each->complexity_;
            add_candidate(base_);
            node->input_ = in;
            base_->complexity_ = base_->complexity_ + in->complexity_ - each->complexity_;
        }
    }

    return make_shared<UnOp>(*node);
}

ast_ptr Perturb::Visit(Bool* node) { return make_shared<Bool>(*node); }

ast_ptr Perturb::Visit(Feature* node) {
  if (node->current_value_ == nullptr) {
    throw invalid_argument("AST has unfilled feature holes");
  } else {
    ast_ptr result = node->current_value_->Accept(this);
    return make_shared<Feature>(*node);
  }
}

ast_ptr Perturb::Visit(Num* node) { return make_shared<Num>(*node); }

ast_ptr Perturb::Visit(Param* node) {
  if (node->current_value_ == nullptr) {
    return make_shared<Param>(*node);
  } else {
    ast_ptr result = node->current_value_->Accept(this);
    return make_shared<Param>(*node);
  }
}

ast_ptr Perturb::Visit(Var* node) {
  return make_shared<Var>(*node);
}

ast_ptr Perturb::Visit(Vec* node) { return make_shared<Vec>(*node); }

}  // namespace AST
