#include <iostream>
#include <memory>

#include "ast.hpp"

using AST::ast_ptr;
using AST::BinOp;
using AST::Num;
using std::cout;
using std::endl;
using std::make_shared;

void PrintAST(ast_ptr root) {
  AST::Print printer;
  root->Accept(&printer);
  printer.Display();
}

int main() {
  Num five(5, {1, 0, 0});
  Num seven(7, {1, 0, 0});
  BinOp adder(make_shared<Num>(five), make_shared<Num>(seven), "Plus");
  BinOp upper(make_shared<Num>(seven), make_shared<BinOp>(adder), "Plus");
  AST::Print printer;
  five.Accept(&printer);
  printer.Display();
  seven.Accept(&printer);
  printer.Display();
  adder.Accept(&printer);
  printer.Display();
  upper.Accept(&printer);
  printer.Display();
  AST::Interp interpreter;
  ast_ptr five_result = five.Accept(&interpreter);
  PrintAST(five_result);
  PrintAST(adder.Accept(&interpreter));
  PrintAST(upper.Accept(&interpreter));
}
