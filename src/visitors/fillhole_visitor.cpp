#include "fillhole_visitor.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "ast/ast.hpp"
#include "deepcopy_visitor.hpp"

using AST::Dimension;
using std::cout;
using std::endl;
using std::make_pair;
using std::make_shared;
using std::pair;
using std::string;
using std::to_string;
using std::unordered_map;
using std::unordered_set;

namespace AST {

unordered_map<string, pair<Type, Dimension>> MapFeatureHoles(
    const ast_ptr& ast) {
  MapHoles mapper;
  ast->Accept(&mapper);
  return mapper.GetFeatureHoles();
}

void FillHoles(ast_ptr& ast, const Model& model) {
  // If there are no holes that still need filling, work is done so exit.
  if (model.empty()) {
    return;
  }

  // Otherwise, just take the first (arbitrary) hole/value pair.
  const auto& p = *(model.cbegin());
  const string& hole_name = p.first;
  const ast_ptr& hole_value = p.second;

  // Do the actual hole filling, using the visitor defined below.
  FillHole filler(hole_name, hole_value);
  ast->Accept(&filler);

  // Create a mutable copy of the model and erase the pair for the filled hole.
  unordered_map<string, ast_ptr> new_model = model;
  new_model.erase(hole_name);

  // Recursive call to fill another hole (if needed).
  FillHoles(ast, new_model);
}

ast_ptr FillHoles(const ast_ptr& ast, const Model& model) {
  ast_ptr copy = DeepCopyAST(ast);
  FillHoles(copy, model);
  return copy;
}

MapHoles::MapHoles() {}

ast_ptr MapHoles::Visit(AST* node) { return ast_ptr(node); }

ast_ptr MapHoles::Visit(BinOp* node) {
  node->left_->Accept(this);
  node->right_->Accept(this);
  return make_shared<BinOp>(*node);
}

ast_ptr MapHoles::Visit(Bool* node) { return make_shared<Bool>(*node); }

ast_ptr MapHoles::Visit(Feature* node) {
  const string& hole_name = node->name_;
  const Type hole_type = node->type_;
  const Dimension hole_dims = node->dims_;
  features_[hole_name] = make_pair(hole_type, hole_dims);
  return make_shared<Feature>(*node);
}

ast_ptr MapHoles::Visit(Num* node) { return make_shared<Num>(*node); }

ast_ptr MapHoles::Visit(Param* node) {
  const string& hole_name = node->name_;
  parameters_.insert(hole_name);
  return make_shared<Param>(*node);
}

ast_ptr MapHoles::Visit(UnOp* node) {
  node->input_->Accept(this);
  return make_shared<UnOp>(*node);
}

ast_ptr MapHoles::Visit(Var* node) { return make_shared<Var>(*node); }

ast_ptr MapHoles::Visit(Vec* node) { return make_shared<Vec>(*node); }

unordered_map<string, pair<Type, Dimension>> MapHoles::GetFeatureHoles() const {
  return features_;
}

unordered_set<string> MapHoles::GetParameterHoles() const {
  return parameters_;
}

void MapHoles::Reset() {
  features_.clear();
  parameters_.clear();
}

FillHole::FillHole(const string& target_name, const ast_ptr& new_value)
    : target_name_(target_name), new_value_(new_value) {}

ast_ptr FillHole::Visit(AST* node) { return ast_ptr(node); }

ast_ptr FillHole::Visit(BinOp* node) {
  node->left_->Accept(this);
  node->right_->Accept(this);
  return make_shared<BinOp>(*node);
}

ast_ptr FillHole::Visit(Bool* node) { return make_shared<Bool>(*node); }

ast_ptr FillHole::Visit(Feature* node) {
  if (node->name_ == target_name_) {
    node->current_value_ = new_value_;
  }
  if (node->current_value_ != nullptr) {
    node->current_value_->Accept(this);
  }
  return make_shared<Feature>(*node);
}

ast_ptr FillHole::Visit(Num* node) { return make_shared<Num>(*node); }

ast_ptr FillHole::Visit(Param* node) {
  if (node->name_ == target_name_) {
    node->current_value_ = new_value_;
  }
  if (node->current_value_ != nullptr) {
    node->current_value_->Accept(this);
  }
  return make_shared<Param>(*node);
}

ast_ptr FillHole::Visit(UnOp* node) {
  node->input_->Accept(this);
  return make_shared<UnOp>(*node);
}

ast_ptr FillHole::Visit(Var* node) { return make_shared<Var>(*node); }

ast_ptr FillHole::Visit(Vec* node) { return make_shared<Vec>(*node); }

}  // namespace AST
