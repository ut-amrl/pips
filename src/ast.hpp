#include <eigen3/Eigen/Core>
#include <memory>
#include <string>

#ifndef SRC_AST_HPP_
#define SRC_AST_HPP_

namespace AST {

enum Type { NODE, NUM, VECTOR, OP };

class AST {
 public:
  AST(Eigen::Vector3i dims, Type type);
  virtual std::shared_ptr<AST> Accept(class Visitor* v) = 0;
  virtual ~AST() = 0;
  Eigen::Vector3i dims_;
  Type type_;

 private:
};

// TypeDef makes life easier
typedef std::shared_ptr<AST> ast_ptr;

class Num : public AST, public std::enable_shared_from_this<Num> {
 public:
  Num(const float& value, Eigen::Vector3i dims);
  ast_ptr Accept(class Visitor* v);
  float value_;

 private:
};
typedef std::shared_ptr<Num> num_ptr;

class BinOp : public AST {
 public:
  BinOp(ast_ptr left, ast_ptr right, const std::string& op);
  ast_ptr Accept(class Visitor* v);
  ast_ptr left_;
  ast_ptr right_;
  const std::string op_;
  Type type_ = OP;

 private:
};
typedef std::shared_ptr<BinOp> bin_ptr;

class Visitor {
 public:
  virtual ast_ptr Visit(AST* node) = 0;
  virtual ast_ptr Visit(Num* node) = 0;
  virtual ast_ptr Visit(BinOp* node) = 0;
};

class Interp : public Visitor {
 public:
  ast_ptr Visit(AST* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(BinOp* node);

 private:
};

class Print : public Visitor {
 public:
  ast_ptr Visit(AST* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(Num* node);
  void Display();

 private:
  std::string program_ = "";
};

}  // namespace AST

#endif  // SRC_AST_HPP
