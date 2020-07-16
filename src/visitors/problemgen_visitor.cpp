#include "problemgen_visitor.hpp"

#include "../ast.hpp"

#include <iostream>  // cout, endl
#include <memory>    // make_shared
#include <stdexcept> // invalid_argument
#include <string>    // string, to_string
#include <vector>    // vector

using std::cout;
using std::endl;
using std::invalid_argument;
using std::make_shared;
using std::string;
using std::to_string;
using std::vector;

namespace AST {

// ProblemGen visitor
ProblemGen::ProblemGen(vector<Example>& examples) : examples_(examples) {
  for (auto _ : examples_) {
    assertions_.push_back("");
  }
}

ast_ptr ProblemGen::Visit(AST* node) { return ast_ptr(node); }

ast_ptr ProblemGen::Visit(BinOp* node) {
  const string op = node->op_;
  string binop_smtlib;
  if (op == "Plus") {
    binop_smtlib += "(+ ";
  } else if (op == "Minus") {
    binop_smtlib += "(- ";
  } else if (op == "Times") {
    binop_smtlib += "(* ";
  } else if (op == "DividedBy") {
    binop_smtlib += "(/ ";
  } else if (op == "Pow") {
    binop_smtlib += "(^ ";
  } else {
    throw invalid_argument("unknown binary operation `" + op + "'");
  }
  for (string& assertion : assertions_) {
    assertion += binop_smtlib;
  }
  node->left_->Accept(this);
  for (string& assertion : assertions_) {
    assertion += " ";
  }
  node->right_->Accept(this);
  for (string& assertion : assertions_) {
    assertion += ")";
  }

  return make_shared<BinOp>(*node);
}

ast_ptr ProblemGen::Visit(Num* node) {
  const string num_string = to_string(node->value_);
  for (string& assertion : assertions_) {
    assertion += num_string;
  }
  return make_shared<Num>(*node);
}

ast_ptr ProblemGen::Visit(Param* node) {
  const string param_name = node->name_;
  for (string& assertion : assertions_) {
    assertion += param_name;
  }
  parameters_.insert(param_name);
  return make_shared<Param>(*node);
}

ast_ptr ProblemGen::Visit(UnOp* node) {
  const string op = node->op_;
  string unop_smtlib;
  if (op == "Abs") {
    unop_smtlib += "(abs ";
  } else if (op == "Cos") {
    unop_smtlib += "(cos ";
  } else if (op == "Sin") {
    unop_smtlib += "(sin ";
  } else {
    throw invalid_argument("unknown unary operation `" + op + "'");
  }
  for (string& assertion : assertions_) {
    assertion += unop_smtlib;
  }
  node->input_->Accept(this);
  for (string& assertion : assertions_) {
    assertion += ")";
  }

  return make_shared<UnOp>(*node);
}

ast_ptr ProblemGen::Visit(Var* node) {
  for (size_t i = 0; i < examples_.size(); ++i) {
    const string var_name = node->name_;
    // Can this be made const?
    Example& example = examples_[i];
    if (example.symbol_table_.find(var_name) != example.symbol_table_.end()) {
      string& assertion = assertions_[i];
      assertion += to_string(example.symbol_table_[var_name].GetFloat());
    } else {
      // TODO(simon) fail harder
      cout << "Variable not found in table!!!" << endl;
    }
  }
  return make_shared<Var>(*node);
}

ast_ptr ProblemGen::Visit(Vec* node) {
  throw invalid_argument("vectors are not yet supported");
}

string ProblemGen::Get() {
  string ret;

  for (const string& param_name : parameters_) {
    ret += "(declare-fun " + param_name + " () Real)\n";
  }

  for (size_t i = 0; i < assertions_.size(); ++i) {
    const string& assertion = assertions_[i];
    const Example& example = examples_[i];

    ret += "(assert (= ";
    ret += to_string(example.result_.GetFloat());
    ret += " ";
    ret += assertion;
    ret += "))\n";
  }

  assertions_.clear();
  examples_.clear();
  parameters_.clear();

  return ret;
}

} // namespace AST