#pragma once

#include "../ast.hpp"

namespace AST {

class Interp : public Visitor {
 public:
  Interp();
  Interp(const Example& world);
  ast_ptr Visit(AST* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(Param* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(Var* node);
  ast_ptr Visit(Vec* node);

 private:
  Example world_;
};

} // namespace AST