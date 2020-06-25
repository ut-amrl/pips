#include <iostream>
#include <memory>

#include "ast.hpp"

using AST::ast_ptr;
using AST::BinOp;
using AST::Example;
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

int main(int argc, char* argv[]) {
  Num five(5, {1, 0, 0});
  Num seven(7, {1, 0, 0});
  Num x(6.1, {1, -1, 0});
  Num y(8.2, {-1, 0, 1});
  Num z(9.3, {1, 0, -1});
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

  Num NegTen(-10, {1, 0, 0});
  Num BadZero(1.4, {1, 1, 1});

  // Adding Library Function Definitions
  // TODO(jaholtz) load these libraries from a file
  vector<FunctionEntry> library;
  FunctionEntry abs_entry;
  abs_entry.input_types_.push_back(AST::NUM);
  abs_entry.input_dims_.push_back({1, 0, 0});
  abs_entry.output_type_ = AST::NUM;
  abs_entry.output_dim_ = {1, 0, 0};
  abs_entry.op_ = "Absolute";
  FunctionEntry abs_entry2;
  abs_entry2.input_types_.push_back(AST::NUM);
  abs_entry2.input_types_.push_back(AST::NUM);
  abs_entry2.input_dims_.push_back({1, 0, 0});
  abs_entry2.input_dims_.push_back({1, 0, 0});
  abs_entry2.output_type_ = AST::NUM;
  abs_entry2.output_dim_ = {1, 0, 0};
  abs_entry2.op_ = "Plus";

  library.push_back(abs_entry2);
  library.push_back(abs_entry);
  // Create inputs
  vector<ast_ptr> inputs, roots;
  roots.push_back(make_shared<Num>(NegTen));
  roots.push_back(make_shared<Num>(BadZero));
  roots.push_back(make_shared<Num>(five));
  roots.push_back(make_shared<Num>(seven));
  inputs.push_back(make_shared<Num>(five));
  inputs.push_back(make_shared<Num>(seven));
  inputs.push_back(make_shared<Num>(BadZero));
  // inputs.push_back(make_shared<Num>(NegTen));
  inputs.push_back(make_shared<Num>(x));
  inputs.push_back(make_shared<Num>(y));
  inputs.push_back(make_shared<Num>(z));
  // library.push_back(abs_entry2);
  vector<vector<float>> signatures;
  vector<Example> examples;
  Example example;
  examples.push_back(example);
  vector<ast_ptr> ops = AST::RecEnumerate(roots, inputs, examples, library,
                                          std::stoi(argv[1]), &signatures);
  cout << "----Roots----" << endl;
  for (auto& node : roots) {
    PrintAST(node);
  }
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
  cout << endl;
  cout << "---- Interp Result ----" << endl;
  for (auto& op : ops) {
    PrintAST(op->Accept(&interpreter));
  }
  cout << endl;
  cout << "---- Number Enumerated ----" << endl;
  cout << ops.size() << endl;
}
