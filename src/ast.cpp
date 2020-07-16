#include "ast.hpp"

#include <eigen3/Eigen/src/Core/Matrix.h>

#include <exception>
#include <iostream>
#include <ostream>
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
using std::ostream;
using std::string;
using std::to_string;
using std::vector;

namespace AST {

ValueProxy::ValueProxy(SymEntry const* owner) : owner_(owner) {}

ValueProxy::operator float() const { return owner_->GetFloat(); }

ValueProxy::operator Vector2f() const { return owner_->GetVector(); }

SymEntry::SymEntry() : float_value_(0), type_(NUM) {}

SymEntry::SymEntry(const bool value) : bool_value_(value), type_(BOOL) {}

SymEntry::SymEntry(const float value) : float_value_(value), type_(NUM) {}

SymEntry::SymEntry(const Eigen::Vector2f& value)
    : vec_value_(value), type_(VEC) {}

ValueProxy SymEntry::GetValue() { return ValueProxy(this); }

const bool SymEntry::GetBool() const {
  if (type_ != BOOL) {
    throw invalid_argument(
        "Attempted to get Boolean "
        "value from non-Boolean symbol.");
  }
  return bool_value_;
}

const float SymEntry::GetFloat() const {
  if (type_ != NUM) {
    throw invalid_argument(
        "Attempted to get float "
        "value from non-float symbol.");
  }
  return float_value_;
}

const Vector2f SymEntry::GetVector() const {
  if (type_ != VEC) {
    throw invalid_argument(
        "Attempted to get vector "
        "value from non-vector symbol.");
  }
  return vec_value_;
}

const Type SymEntry::GetType() const { return type_; }

bool SymEntry::operator==(const SymEntry& other) const {
  if (type_ != other.type_) {
    return false;
  }

  switch (type_) {
    case BOOL:
      return bool_value_ == other.bool_value_;
    case NUM:
      return float_value_ == other.float_value_;
    case VEC:
      return vec_value_ == other.vec_value_;
    default:
      throw invalid_argument("Invalid SymEntry");
  }
}

ostream& operator<<(ostream& stream, const SymEntry& symentry) {
  switch (symentry.type_) {
    case BOOL:
      stream << symentry.GetBool();
      break;
    case NUM:
      stream << symentry.GetFloat();
      break;
    case VEC:
      stream << symentry.GetVector();
      break;
    default:
      throw invalid_argument("Invalid SymEntry");
  }
  return stream;
}

bool Example::operator==(const Example& other) const {
  return result_ == other.result_ && symbol_table_ == other.symbol_table_;
}

std::ostream& operator<<(std::ostream& stream, const Example& example) {
  stream << "in:";
  for (auto& name_value : example.symbol_table_) {
    const string& name = name_value.first;
    const SymEntry& value = name_value.second;
    stream << "\t" << name << "=" << value;
  }
  stream << ",\tout:\t" << example.result_;
  return stream;
}

// Constructors
AST::AST(const Dimension& dims, const Type& type) : dims_(dims), type_(type) {}

AST::~AST(){};

BinOp::BinOp(ast_ptr left, ast_ptr right, const string& op)
    : AST({0, 0, 0}, OP), left_(left), right_(right), op_(op) {}

BinOp::BinOp(ast_ptr left, ast_ptr right, const string& op, const Type& type,
             const Dimension& dim)
    : AST(dim, type), left_(left), right_(right), op_(op) {}

Bool::Bool(const bool& value) : AST({0, 0, 0}, BOOL), value_(value) {}

Feature::Feature(const string& name, const Dimension& dims, const Type& type)
    : AST(dims, type), name_(name) {}

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
ast_ptr Bool::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Feature::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Num::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Param::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr UnOp::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Var::Accept(class Visitor* v) { return v->Visit(this); }
ast_ptr Vec::Accept(class Visitor* v) { return v->Visit(this); }
// End Casting Calls

bool Var::operator==(const Var& other) const {
  return type_ == other.type_ && dims_ == other.dims_ && name_ == other.name_;
}

}  // namespace AST
