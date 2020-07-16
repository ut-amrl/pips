#include <eigen3/Eigen/Core>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "nlohmann/json.hpp"

#ifndef SRC_AST_HPP_
#define SRC_AST_HPP_

namespace AST {

class SymEntry;

class ValueProxy {
 public:
  ValueProxy(SymEntry const* owner);
  operator float() const;
  operator Eigen::Vector2f() const;

 private:
  SymEntry const* owner_;
};

class SymEntry {
 public:
  SymEntry();
  SymEntry(const float value);
  SymEntry(const Eigen::Vector2f& value);
  ValueProxy GetValue();
  const float GetFloat() const;
  const Eigen::Vector2f GetVector() const;
  std::string name_;

 private:
  float float_value_;
  Eigen::Vector2f vec_value_;
  bool is_num_;
};

struct Example {
  std::map<std::string, SymEntry> symbol_table_;
  SymEntry result_;
};

enum Type { NODE, VAR, NUM, VEC, OP };

// useful typedef for tracking purpose
typedef Eigen::Vector3i Dimension;

class AST {
 public:
  AST(const Dimension& dims, const Type& type);
  virtual std::shared_ptr<AST> Accept(class Visitor* v) = 0;
  virtual ~AST() = 0;
  const Dimension dims_;
  const Type type_;

 private:
};
typedef std::shared_ptr<AST> ast_ptr;

class Var : public AST {
 public:
  Var(const std::string& name, const Dimension& dims, const Type& type);
  ast_ptr Accept(class Visitor* v);
  const std::string name_;

 private:
};
typedef std::shared_ptr<Var> var_ptr;

class Param : public AST {
 public:
  Param(const std::string& name, const Dimension& dims, const Type& type);
  ast_ptr Accept(class Visitor* v);
  const std::string name_;

 private:
};

class Num : public AST {
 public:
  Num(const float& value, const Dimension& dims);
  ast_ptr Accept(class Visitor* v);
  float value_;

 private:
};
typedef std::shared_ptr<Num> num_ptr;

class UnOp : public AST {
 public:
  UnOp(ast_ptr input, const std::string& op);
  UnOp(ast_ptr input, const std::string& op, const Type& type,
       const Dimension& dim);
  ast_ptr Accept(class Visitor* v);
  ast_ptr input_;
  const std::string op_;
  Type type_ = OP;

 private:
};
typedef std::shared_ptr<UnOp> un_ptr;

class BinOp : public AST {
 public:
  BinOp(ast_ptr left, ast_ptr right, const std::string& op);
  BinOp(ast_ptr left, ast_ptr right, const std::string& op, const Type& type,
        const Dimension& dim);
  ast_ptr Accept(class Visitor* v);
  ast_ptr left_;
  ast_ptr right_;
  const std::string op_;
  Type type_ = OP;

 private:
};
typedef std::shared_ptr<BinOp> bin_ptr;

class Vec : public AST {
 public:
  Vec(Eigen::Vector2f value, Eigen::Vector3i dims);
  ast_ptr Accept(class Visitor* v);
  Eigen::Vector2f value_;
  Type type_ = VEC;

 private:
};
typedef std::shared_ptr<Vec> vec_ptr;

class Visitor {
 public:
  virtual ast_ptr Visit(AST* node) = 0;
  virtual ast_ptr Visit(Var* node) = 0;
  virtual ast_ptr Visit(Param* node) = 0;
  virtual ast_ptr Visit(Num* node) = 0;
  virtual ast_ptr Visit(UnOp* node) = 0;
  virtual ast_ptr Visit(BinOp* node) = 0;
  virtual ast_ptr Visit(Vec* node) = 0;
};

}  // namespace AST
#endif  // SRC_AST_HPP
