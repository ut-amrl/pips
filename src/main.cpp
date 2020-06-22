#include <iostream>
#include <memory>

#include "ast.hpp"

using AST::ast_ptr;
using AST::BinOp;
using AST::FunctionEntry;
using AST::Num;
using std::cout;
using std::endl;
using std::make_shared;
using std::vector;

void PrintAST(ast_ptr root) {
  AST::Print printer;
  root->Accept(&printer);
  printer.Display();
}

int main() {
  Num five(5, {1, 0, 0});
  Num seven(7, {1, 0, 0});
  Num x(6, {1, -1, 0});
  Num y(8, {-1, 0, 1});
  Num z(7, {1, 0, -1});
  BinOp adder(make_shared<Num>(five), make_shared<Num>(seven), "Plus");
  BinOp upper(make_shared<Num>(seven), make_shared<BinOp>(adder), "Plus");
  // Simple Example
  cout << "---- Basic AST Evaluation Example ----" << endl << endl;
  AST::Interp interpreter;

  cout << "---- Numeric Type (with Dimension) ----" << endl;
  PrintAST(make_shared<Num>(five));
  PrintAST(make_shared<Num>(x));
  PrintAST(make_shared<Num>(seven));

  cout << endl;
  cout << "---- Function Nesting No Interp ----" << endl;
  PrintAST(make_shared<BinOp>(adder));
  PrintAST(make_shared<BinOp>(upper));

  cout << endl;
  cout << "---- Functions Interpreted ----" << endl;

  PrintAST(adder.Accept(&interpreter));
  PrintAST(upper.Accept(&interpreter));

  cout << endl;
  cout << "---- Basic Enumeration Example ----" << endl << endl;
  ;
  Num NegTen(-10, {1, 0, 0});
  Num BadZero(0, {1, 1, 1});
  // Adding Library Function Definitions
  vector<FunctionEntry> library;
  FunctionEntry abs_entry;
  abs_entry.input_types_.push_back(AST::NUM);
  abs_entry.input_dims_.push_back({1, 0, 0});
  abs_entry.op_ = "Absolute";
  FunctionEntry abs_entry2;
  abs_entry2.input_types_.push_back(AST::NUM);
  abs_entry2.input_types_.push_back(AST::NUM);
  abs_entry2.input_dims_.push_back({1, 0, 0});
  abs_entry2.input_dims_.push_back({1, 0, 0});
  abs_entry2.op_ = "Plus";
  // FunctionEntry abs_entry3;
  // abs_entry3.input_types_.push_back(AST::NUM);
  // abs_entry3.input_dims_.push_back({1,0,0});
  // abs_entry3.op_ = "Absolute";
  library.push_back(abs_entry2);
  library.push_back(abs_entry);
  // Create inputs
  vector<ast_ptr> inputs;
  inputs.push_back(make_shared<Num>(five));
  inputs.push_back(make_shared<Num>(seven));
  inputs.push_back(make_shared<Num>(x));
  inputs.push_back(make_shared<Num>(y));
  inputs.push_back(make_shared<Num>(z));
  // library.push_back(abs_entry2);
  vector<ast_ptr> ops =
      AST::GetLegalOps(make_shared<Num>(NegTen), inputs, library);
  cout << "----Node----" << endl;
  PrintAST(make_shared<Num>(NegTen));
  cout << endl;
  cout << "----Library----" << endl;
  for (auto& func : library) {
    cout << func.op_ << endl;
  }
  cout << endl;
  cout << "----Inputs----" << endl;
  for (auto& input : inputs) {
    PrintAST(input);
  }
  cout << endl;
  cout << "----Result----" << endl;
  for (auto& op : ops) {
    PrintAST(op);
  }
}
