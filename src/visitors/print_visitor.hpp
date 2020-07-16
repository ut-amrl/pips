#pragma once

#include "../ast.hpp"

namespace AST {

class Print : public Visitor {
 public:
  ast_ptr Visit(AST* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(Param* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(Var* node);
  ast_ptr Visit(Vec* node);
  void Display();

 private:
  std::string program_ = "";
  int depth_ = 0;
};

}  // namespace AST