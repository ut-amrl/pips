#pragma once

#include <ostream>

#include "ast/ast.hpp"

namespace AST {

std::vector<bool> TraceOn(const ast_ptr& ast);

class Trace : public Visitor {
 public:
  ast_ptr Visit(AST* node);
  ast_ptr Visit(If* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(Bool* node);
  ast_ptr Visit(String* node);
  ast_ptr Visit(Feature* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(Param* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(Var* node);
  ast_ptr Visit(Vec* node);
  std::vector<bool> GetTrace() const;

 private:
  std::vector<bool> trace_;
};

}  // namespace AST
