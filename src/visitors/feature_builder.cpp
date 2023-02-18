#include "feature_builder.hpp"
#include "deepcopy_visitor.hpp"
#include "print_visitor.hpp"
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

vector<ast_ptr> build_feat(ast_ptr& base, vector<ast_ptr>& lib) {
    vector<ast_ptr> extended_lib;
    for(ast_ptr each: lib) {
        vector<ast_ptr> empty;
        vector<ast_ptr> feats = findSimilar(each, lib, empty);
        extended_lib.insert(extended_lib.end(), feats.begin(), feats.end());
    }

    Feature_Builder visitor(lib);
    base->Accept(&visitor);

    for(ast_ptr each: extended_lib) {
        visitor.base_ = each;
        each->Accept(&visitor);
    }

    cout << "| Extended Features Count: " << visitor.features.size() << endl;
    return visitor.features;
}

void Feature_Builder::add_candidate(ast_ptr prog) { 
    ast_ptr copy = DeepCopyAST(prog);
    string s = GetString(copy);
    if(hashes_.count(s) == 0){
        hashes_.insert(s);
        features.push_back(copy);
    }
}

Feature_Builder::Feature_Builder(vector<ast_ptr>& lib) : lib_(lib) {
    for(ast_ptr each: lib) {
        add_candidate(each);
    }
}

ast_ptr Feature_Builder::Visit(AST* node) { return ast_ptr(node); }

ast_ptr Feature_Builder::Visit(TernOp* node){
    const string op = node->op_;
    if(op == "Logistic") {
        base_feat.push_back(node->x_);
        return make_shared<TernOp>(*node);
    } else {
        // TODO: additional ternary support
    }

    ast_ptr x = node->x_->Accept(this);
    ast_ptr a = node->a_->Accept(this);
    ast_ptr b = node->b_->Accept(this);

    return make_shared<TernOp>(*node);
}

ast_ptr Feature_Builder::Visit(BinOp* node) {
    const string op = node->op_;
    if(op == "And" || op == "Or" || op == "Flip") {
        ast_ptr left = node->left_->Accept(this);
        ast_ptr right = node->right_->Accept(this);
    }

    ast_ptr c_left = node->left_;
    ast_ptr c_right = node->right_;
    for(ast_ptr each: base_feat) {
        if(each->dims_ == c_left->dims_) {
            node->left_ = each;
            base_->complexity_ = base_->complexity_ - c_left->complexity_ + each->complexity_;
            add_candidate(base_);
            node->left_ = c_left;
            base_->complexity_ = base_->complexity_ + c_left->complexity_ - each->complexity_;
        }

        if(each->dims_ == c_right->dims_) {
            node->right_ = each;
            base_->complexity_ = base_->complexity_ - c_right->complexity_ + each->complexity_;
            add_candidate(base_);
            node->right_ = c_right;
            base_->complexity_ = base_->complexity_ + c_right->complexity_ - each->complexity_;
        }
    }

    return make_shared<BinOp>(*node);
}

ast_ptr Feature_Builder::Visit(UnOp* node) {
    const string op = node->op_;

    ast_ptr in = node->input_;
    for(ast_ptr each: base_feat) {
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

ast_ptr Feature_Builder::Visit(Bool* node) { return make_shared<Bool>(*node); }

ast_ptr Feature_Builder::Visit(Feature* node) {
  if (node->current_value_ == nullptr) {
    throw invalid_argument("AST has unfilled feature holes");
  } else {
    ast_ptr result = node->current_value_->Accept(this);
    return make_shared<Feature>(*node);
  }
}

ast_ptr Feature_Builder::Visit(Num* node) { return make_shared<Num>(*node); }

ast_ptr Feature_Builder::Visit(Param* node) {
  if (node->current_value_ == nullptr) {
    return make_shared<Param>(*node);
  } else {
    ast_ptr result = node->current_value_->Accept(this);
    return make_shared<Param>(*node);
  }
}

ast_ptr Feature_Builder::Visit(Var* node) {
  return make_shared<Var>(*node);
}

ast_ptr Feature_Builder::Visit(Vec* node) { return make_shared<Vec>(*node); }

}  // namespace AST
