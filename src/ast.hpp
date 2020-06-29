#include <eigen3/Eigen/Core>
#include <map>
#include <memory>
#include <string>
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
  virtual ast_ptr Visit(Num* node) = 0;
  virtual ast_ptr Visit(UnOp* node) = 0;
  virtual ast_ptr Visit(BinOp* node) = 0;
  virtual ast_ptr Visit(Vec* node) = 0;
};

class Interp : public Visitor {
 public:
  Interp();
  Interp(const Example& world);
  ast_ptr Visit(AST* node);
  ast_ptr Visit(Var* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(Vec* node);

 private:
  Example world_;
};

class Print : public Visitor {
 public:
  ast_ptr Visit(AST* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(Var* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(Vec* node);
  void Display();

 private:
  std::string program_ = "";
  int depth_ = 0;
};

struct FunctionSig {
  std::vector<Type> input_types_;
  std::vector<Dimension> input_dims_;
  Type output_type_;
  Dimension output_dim_;
};

struct FunctionEntry {
  std::string op_;
  std::vector<Type> input_types_;
  std::vector<Dimension> input_dims_;
  Type output_type_;
  Dimension output_dim_;
};

std::vector<ast_ptr> GetLegalOps(ast_ptr node, std::vector<ast_ptr> input,
                                 const std::vector<FunctionEntry>& library);

std::vector<ast_ptr> Enumerate(const std::vector<ast_ptr>& roots,
                               const std::vector<ast_ptr>& inputs,
                               const std::vector<FunctionEntry>& library);

std::vector<ast_ptr> RecEnumerate(const std::vector<ast_ptr>& roots,
                                  const std::vector<ast_ptr>& inputs,
                                  const std::vector<Example>& examples,
                                  const std::vector<FunctionEntry>& library,
                                  const int depth,
                                  std::vector<std::vector<float>>* signatures);

template <typename T>
bool IndexInVector(const std::vector<T>& vec, const T& element, int* index) {
  // Find given element in vector
  auto it = std::find(vec.begin(), vec.end(), element);
  if (it != vec.end()) {
    *index = distance(vec.begin(), it);
    return true;
  }
  *index = -1;
  return false;
}

}  // namespace AST
#endif  // SRC_AST_HPP
