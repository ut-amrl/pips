#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>

#include "../ast.hpp"
#include "../enumeration.hpp"

namespace AST {

std::unordered_map<std::string, Type> MapFeatureHoles(const ast_ptr& ast);
void FillHoles(ast_ptr& ast, const Model& model);
ast_ptr FillHoles(const ast_ptr& ast, const Model& model);

class MapHoles : public Visitor {
 public:
  MapHoles();
  ast_ptr Visit(AST* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(Bool* node);
  ast_ptr Visit(Feature* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(Param* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(Var* node);
  ast_ptr Visit(Vec* node);
  std::unordered_map<std::string, Type> GetFeatureHoles() const;
  std::unordered_set<std::string> GetParameterHoles() const;
  void Reset();

 private:
  std::unordered_map<std::string, Type> features_;
  std::unordered_set<std::string> parameters_;
};

class FillHole : public Visitor {
 public:
  FillHole(const std::string& target_name, const ast_ptr& new_value);
  ast_ptr Visit(AST* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(Bool* node);
  ast_ptr Visit(Feature* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(Param* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(Var* node);
  ast_ptr Visit(Vec* node);

 private:
  const std::string target_name_;
  const ast_ptr new_value_;
};

}  // namespace AST