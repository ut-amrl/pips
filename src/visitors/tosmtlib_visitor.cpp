#include "tosmtlib_visitor.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "../ast.hpp"

using std::cout;
using std::endl;
using std::invalid_argument;
using std::make_shared;
using std::string;
using std::to_string;
using std::unordered_set;
using std::vector;

namespace AST {

ToSMTLIB::ToSMTLIB(const Example& example) : example_(example) {}

ast_ptr ToSMTLIB::Visit(AST* node) { return ast_ptr(node); }

ast_ptr ToSMTLIB::Visit(BinOp* node) {
  const string op = node->op_;
  string binop_smtlib;
  if (op == "Plus") {
    binop_smtlib += "(+ ";
  } else if (op == "Minus") {
    binop_smtlib += "(- ";
  } else if (op == "Times") {
    binop_smtlib += "(* ";
  } else if (op == "DividedBy") {
    binop_smtlib += "(safe-div ";
  } else if (op == "Pow") {
    binop_smtlib += "(^ ";
  } else if (op == "Cross") {
    binop_smtlib += "(cross ";
  } else if (op == "Dot") {
    binop_smtlib += "(dot ";
  } else if (op == "EuclideanDistanceSq") {
    binop_smtlib += "(euc-dist-sq ";
  } else if (op == "And") {
    binop_smtlib += "(and ";
  } else if (op == "Or") {
    binop_smtlib += "(or ";
  } else if (op == "Eq") {
    binop_smtlib += "(= ";
  } else if (op == "Gt") {
    binop_smtlib += "(> ";
  } else if (op == "Lt") {
    binop_smtlib += "(< ";
  } else if (op == "Gte") {
    binop_smtlib += "(>= ";
  } else if (op == "Lte") {
    binop_smtlib += "(<= ";
  } else {
    throw invalid_argument("unknown binary operation `" + op + "'");
  }
  output_ += binop_smtlib;
  node->left_->Accept(this);
  output_ += " ";
  node->right_->Accept(this);
  output_ += ")";

  return make_shared<BinOp>(*node);
}

ast_ptr ToSMTLIB::Visit(Bool* node) {
  const string bool_string = (node->value_) ? "true" : "false";
  output_ += bool_string;
  return make_shared<Bool>(*node);
}

ast_ptr ToSMTLIB::Visit(Feature* node) {
  if (node->current_value_ != nullptr) {
    // TODO(simon) delete next two lines maybe?
    const string feature_name = node->name_;
    features_.insert(feature_name);
    node->current_value_->Accept(this);
    return make_shared<Feature>(*node);
  } else {
    throw invalid_argument(
        "attempted to make SMTLIB "
        "from AST with feature holes");
  }
}

ast_ptr ToSMTLIB::Visit(Num* node) {
  const string num_string = to_string(node->value_);
  output_ += num_string;
  return make_shared<Num>(*node);
}

ast_ptr ToSMTLIB::Visit(Param* node) {
  const string param_name = node->name_;
  output_ += param_name;
  parameters_.insert(param_name);
  return make_shared<Param>(*node);
}

ast_ptr ToSMTLIB::Visit(UnOp* node) {
  const string op = node->op_;
  string unop_smtlib;
  if (op == "Abs") {
    unop_smtlib += "(abs ";
  } else if (op == "Sq") {
    unop_smtlib += "(sq ";
  } else if (op == "Cos") {
    unop_smtlib += "(cos ";
  } else if (op == "Sin") {
    unop_smtlib += "(sin ";
  } else if (op == "NormSq") {
    unop_smtlib += "(norm-sq ";
  } else if (op == "VecX") {
    unop_smtlib += "(vec-x ";
  } else if (op == "VecY") {
    unop_smtlib += "(vec-y ";
  } else if (op == "Not") {
    unop_smtlib += "(not ";
  } else {
    throw invalid_argument("unknown unary operation `" + op + "'");
  }
  output_ += unop_smtlib;
  output_ += " ";
  node->input_->Accept(this);
  output_ += ")";

  return make_shared<UnOp>(*node);
}

ast_ptr ToSMTLIB::Visit(Var* node) {
  const string var_name = node->name_;
  auto it = example_.symbol_table_.find(var_name);
  if (it != example_.symbol_table_.end()) {
    const SymEntry se = (*it).second;
    switch (se.GetType()) {
      case BOOL:
        output_ += (se.GetBool() ? "true" : "false");
        break;
      case NUM:
        output_ += to_string(se.GetFloat());
        break;
      case VEC:
        output_ +=
            to_string(se.GetVector().x()) + " " + to_string(se.GetVector().y());
        break;
      default:
        throw invalid_argument("unknown/unsupported SymEntry type");
    }
    return make_shared<Var>(*node);
  } else {
    throw invalid_argument("Variable not found in symbol table");
  }
}

ast_ptr ToSMTLIB::Visit(Vec* node) {
  const string x_string = to_string(node->value_.x());
  const string y_string = to_string(node->value_.y());
  const string vec_string = x_string + " " + y_string;
  output_ += vec_string;
  return make_shared<Vec>(*node);
}

string ToSMTLIB::Get() const { return output_; }

unordered_set<string> ToSMTLIB::GetFeatures() const { return features_; }

unordered_set<string> ToSMTLIB::GetParams() const { return parameters_; }

void ToSMTLIB::Reset() {
  output_.clear();
  parameters_.clear();
}

ToSMTLIB AstToSMTLIB(const ast_ptr& ast, const Example& example) {
  ToSMTLIB converter(example);
  ast->Accept(&converter);
  return converter;
}

}  // namespace AST