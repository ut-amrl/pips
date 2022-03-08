#include "trace_visitor.hpp"
#include "print_visitor.hpp"

#include <iostream>  // cout, endl
#include <memory>    // make_shared
#include <string>    // to_string

using std::cout;
using std::endl;
using std::make_shared;
using std::ostream;
using std::to_string;
const bool kDebug = false;

namespace AST {

ast_ptr Trace::Visit(AST* node) { return ast_ptr(node); }

ast_ptr Trace::Visit(If* node) {
  if (print_ && check_trace_[trace_.size()] == 0) {
    cout << "----" << endl;
    PrintAST(make_shared<If>(*node));
    cout << "Index: " << trace_.size() << endl;
    cout << "Visited: " << node->visited_ << endl;
    cout << "----" << endl;
  }
  trace_.push_back(node->visited_);
  node->cond_->Accept(this);
  node->left_->Accept(this);
  node->right_->Accept(this);
  return make_shared<If>(*node);
}

ast_ptr Trace::Visit(BinOp* node) {
  if (print_ && check_trace_[trace_.size()] == 0) {
    cout << "----" << endl;
    PrintAST(make_shared<BinOp>(*node));
    cout << "Index: " << trace_.size() << endl;
    cout << "Visited: " << node->visited_ << endl;
    cout << "----" << endl;
  }
  trace_.push_back(node->visited_);
  node->left_->Accept(this);
  node->right_->Accept(this);
  return make_shared<BinOp>(*node);
}

ast_ptr Trace::Visit(Bool* node) {
  // if (kDebug) {
    // cout << "----" << endl;
    // PrintAST(make_shared<Bool>(*node));
    // cout << "Index: " << trace_.size() << endl;
    // cout << "Visited: " << node->visited_ << endl;
    // cout << "----" << endl;
  // }
  // trace_.push_back(node->visited_);
  return make_shared<Bool>(*node);
}

ast_ptr Trace::Visit(String* node) {
  if (print_ && check_trace_[trace_.size()] == 0) {
    cout << "----" << endl;
    PrintAST(make_shared<String>(*node));
    cout << "Index: " << trace_.size() << endl;
    cout << "Visited: " << node->visited_ << endl;
    cout << "----" << endl;
  }
  trace_.push_back(node->visited_);
  return make_shared<String>(*node);
}

ast_ptr Trace::Visit(Feature* node) {
  // if (kDebug) {
    // cout << "----" << endl;
    // PrintAST(make_shared<Feature>(*node));
    // cout << "Index: " << trace_.size() << endl;
    // cout << "Visited: " << node->visited_ << endl;
    // cout << "----" << endl;
  // }
  // trace_.push_back(node->visited_);
  return make_shared<Feature>(*node);
}

ast_ptr Trace::Visit(Num* node) {
  // if (kDebug) {
    // cout << "----" << endl;
    // PrintAST(make_shared<Num>(*node));
    // cout << "Index: " << trace_.size() << endl;
    // cout << "Visited: " << node->visited_ << endl;
    // cout << "----" << endl;
  // }
  // trace_.push_back(node->visited_);
  return make_shared<Num>(*node);
}

ast_ptr Trace::Visit(Param* node) {
  // if (kDebug) {
    // cout << "----" << endl;
    // PrintAST(make_shared<Param>(*node));
    // cout << "Index: " << trace_.size() << endl;
    // cout << "Visited: " << node->visited_ << endl;
    // cout << "----" << endl;
  // }
  // trace_.push_back(node->visited_);
  return make_shared<Param>(*node);
}

ast_ptr Trace::Visit(UnOp* node) {
  // if (kDebug) {
    // cout << "----" << endl;
    // PrintAST(make_shared<UnOp>(*node));
    // cout << "Index: " << trace_.size() << endl;
    // cout << "Visited: " << node->visited_ << endl;
    // cout << "----" << endl;
  // }
  // trace_.push_back(node->visited_);
  node->input_->Accept(this);
  return make_shared<UnOp>(*node);
}

ast_ptr Trace::Visit(Var* node) {
  // if (kDebug) {
    // cout << "----" << endl;
    // PrintAST(make_shared<Var>(*node));
    // cout << "Index: " << trace_.size() << endl;
    // cout << "Visited: " << node->visited_ << endl;
    // cout << "----" << endl;
  // }
  // trace_.push_back(node->visited_);
  return make_shared<Var>(*node);
}

ast_ptr Trace::Visit(Vec* node) {
  // if (kDebug) {
    // cout << "----" << endl;
    // PrintAST(make_shared<Vec>(*node));
    // cout << "Index: " << trace_.size() << endl;
    // cout << "Visited: " << node->visited_ << endl;
    // cout << "----" << endl;
  // }
  // trace_.push_back(node->visited_);
  return make_shared<Vec>(*node);
}

std::vector<bool> Trace::GetTrace() const { return trace_; }

std::vector<bool> TraceOn(const ast_ptr& ast) {
  Trace tracer;
  ast->Accept(&tracer);
  return tracer.GetTrace();
}

std::vector<bool> CheckTrace(const ast_ptr& ast,
                             const boost::dynamic_bitset<>& trace) {
  Trace tracer;
  tracer.print_ = true;
  tracer.check_trace_ = trace;
  ast->Accept(&tracer);
  return tracer.GetTrace();
}

}  // namespace AST
