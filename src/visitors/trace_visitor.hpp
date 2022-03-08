#pragma once

#include <ostream>

#include "ast/ast.hpp"
#include "boost/dynamic_bitset.hpp"

namespace AST {

std::vector<bool> TraceOn(const ast_ptr& ast);
std::vector<bool> CheckTrace(const ast_ptr& ast,
                             const boost::dynamic_bitset<>& trace);

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
  boost::dynamic_bitset<> check_trace_;
  bool print_ = false;

 private:
  std::vector<bool> trace_;
};

}  // namespace AST
