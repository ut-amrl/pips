#include "ast.hpp"

#include <eigen3/Eigen/src/Core/Matrix.h>

#include <exception>
#include <iostream>
#include <stdexcept>

#include "library_functions.hpp"
#include "util/timer.h"

using Eigen::Vector2f;
using Eigen::Vector3i;
using nlohmann::json;
using std::cout;
using std::dynamic_pointer_cast;
using std::endl;
using std::invalid_argument;
using std::make_shared;
using std::map;
using std::string;
using std::to_string;
using std::vector;

namespace AST {

ValueProxy::ValueProxy(SymEntry const* owner) : owner_(owner) {}

ValueProxy::operator float() const { return owner_->GetFloat(); }

ValueProxy::operator Vector2f() const { return owner_->GetVector(); }

SymEntry::SymEntry() : float_value_(0), vec_value_({0, 0}), is_num_(false) {}

SymEntry::SymEntry(const float value)
    : float_value_(value), vec_value_({0, 0}), is_num_(true) {}

SymEntry::SymEntry(const Eigen::Vector2f& value)
    : float_value_(0.0), vec_value_(value), is_num_(false) {}

ValueProxy SymEntry::GetValue() { return ValueProxy(this); }

const float SymEntry::GetFloat() const {
  if (!is_num_) {
    throw invalid_argument(
        "Attempted to get float "
        "value from non-float symbol.");
  }
  return float_value_;
}

const Vector2f SymEntry::GetVector() const {
  if (is_num_) {
    throw invalid_argument(
        "Attempted to get vector "
        "value from non-vector symbol.");
  }
  return vec_value_;
}

// Constructors
AST::AST(const Dimension& dims, const Type& type) : dims_(dims), type_(type) {}

AST::~AST(){};

BinOp::BinOp(ast_ptr left, ast_ptr right, const string& op)
    : AST({0, 0, 0}, OP), left_(left), right_(right), op_(op) {}

BinOp::BinOp(ast_ptr left, ast_ptr right, const string& op, const Type& type,
             const Dimension& dim)
    : AST(dim, type), left_(left), right_(right), op_(op) {}

Num::Num(const float& value, const Dimension& dims)
    : AST(dims, NUM), value_(value) {}

Param::Param(const string& name, const Dimension& dims, const Type& type)
    : AST(dims, type), name_(name) {}

UnOp::UnOp(ast_ptr input, const string& op)
    : AST({0, 0, 0}, OP), input_(input), op_(op) {}

UnOp::UnOp(ast_ptr input, const string& op, const Type& type,
           const Dimension& dim)
    : AST(dim, type), input_(input), op_(op) {}

Var::Var(const string& name, const Dimension& dims, const Type& type)
    : AST(dims, type), name_(name) {}

Vec::Vec(Vector2f value, Vector3i dims) : AST(dims, VEC), value_(value) {}

// Necessary to get the automatic casting correct
ast_ptr AST::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr BinOp::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Num::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Param::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr UnOp::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Var::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Vec::Accept(class Visitor* v) { return v->Visit(this); }
// End Casting Calls

}  // namespace AST
