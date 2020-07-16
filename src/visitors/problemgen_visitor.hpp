#pragma once

#include "../ast.hpp"

#include <string>
#include <unordered_set>
#include <vector>

namespace AST {

class ProblemGen : public Visitor {
 public:
  ProblemGen(std::vector<Example>& example);
  ast_ptr Visit(AST* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(Param* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(Var* node);
  ast_ptr Visit(Vec* node);
  std::string Get();

 private:
  std::vector<std::string> assertions_;
  std::vector<Example> examples_;
  std::unordered_set<std::string> parameters_;
};

} // namespace AST