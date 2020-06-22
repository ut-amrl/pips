#include <eigen3/Eigen/Core>
#include <map>
#include <memory>
#include <string>
#include <vector>

#ifndef SRC_AST_HPP_
#define SRC_AST_HPP_

namespace AST {

enum Type { NODE, NUM, VECTOR, OP };

// useful typedef for tracking purpose
typedef Eigen::Vector3i dim;

class AST {
 public:
  AST(Eigen::Vector3i dims, Type type);
  virtual std::shared_ptr<AST> Accept(class Visitor* v) = 0;
  virtual ~AST() = 0;
  dim dims_;
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

class UnOp : public AST {
 public:
  UnOp(ast_ptr input, const std::string& op);
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
  virtual ast_ptr Visit(UnOp* node) = 0;
  virtual ast_ptr Visit(BinOp* node) = 0;
};

class Interp : public Visitor {
 public:
  ast_ptr Visit(AST* node);
  ast_ptr Visit(Num* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(BinOp* node);

 private:
};

class Print : public Visitor {
 public:
  ast_ptr Visit(AST* node);
  ast_ptr Visit(UnOp* node);
  ast_ptr Visit(BinOp* node);
  ast_ptr Visit(Num* node);
  void Display();

 private:
  std::string program_ = "";
  int depth_ = 0;
};

struct FunctionSig {
  std::vector<Type> input_types_;
  std::vector<dim> input_dims_;
  Type output_type_;
  dim output_dim_;
};

struct FunctionEntry {
  std::string op_;
  std::vector<Type> input_types_;
  std::vector<Eigen::Vector3i> input_dims_;
  Type output_type_;
  dim output_dim_;
};

// class Enumerate : public Visitor {
// public:
// ast_ptr Visit(AST* node);
// ast_ptr Visit(BinOp* node);
// ast_ptr Visit(Num* node);

// private:
// std::string program_ = "";
// std::vector<ast_ptr> features_;
// std::vector<float> num_sigs_;
// std::vector<FunctionEntry> library_;
// };

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

std::vector<ast_ptr> GetLegalOps(ast_ptr node, std::vector<ast_ptr> input,
                                 const std::vector<FunctionEntry>& library);

}  // namespace AST

#endif  // SRC_AST_HPP
