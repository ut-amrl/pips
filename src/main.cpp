#include <z3++.h>

#include <fstream>
#include <iostream>
#include <memory>

#include "ast.hpp"
#include "nlohmann/json.hpp"

using AST::ast_ptr;
using AST::BinOp;
using AST::Dimension;
using AST::Example;
using AST::FunctionEntry;
using AST::Num;
using AST::NUM;
using AST::Param;
using AST::SymEntry;
using AST::Type;
using AST::Var;
using AST::VEC;
using Eigen::Vector2f;
using std::cout;
using std::endl;
using std::invalid_argument;
using std::make_shared;
using std::map;
using std::string;
using std::vector;
using json = nlohmann::json;
using z3::context;
using z3::solver;

void PrintAST(ast_ptr root) {
  AST::Print printer;
  root->Accept(&printer);
  printer.Display();
}

string MakeSMTLIBProblem(ast_ptr& root, vector<Example>& examples) {
  AST::ProblemGen gen(examples);
  root->Accept(&gen);
  return gen.Get();
}

Type StringToType(const string& type_string) {
  if (type_string == "NODE") {
    return Type::NODE;
  } else if (type_string == "VAR") {
    return Type::VAR;
  } else if (type_string == "NUM") {
    return Type::NUM;
  } else if (type_string == "VEC") {
    return Type::VEC;
  } else if (type_string == "OP") {
    return Type::OP;
  }
  return Type::NODE;
}

vector<FunctionEntry> ReadLibrary(const string& file) {
  vector<FunctionEntry> result;
  std::ifstream input(file);
  json library;
  input >> library;
  for (auto& func : library) {
    FunctionEntry entry;
    entry.op_ = func["op"];
    const vector<int> out_dim = func["outputDim"];
    const string out_type = func["outputType"];
    entry.output_dim_ = Dimension(out_dim.data());
    entry.output_type_ = StringToType(out_type);
    for (const string in : func["inputType"]) {
      entry.input_types_.push_back(StringToType(in));
    }
    for (const vector<int> in : func["inputDim"]) {
      entry.input_dims_.push_back(Dimension(in.data()));
    }
    result.push_back(entry);
  }
  return result;
}

// Parse json examples file into examples and create
// variables for world inputs.
vector<Example> ReadExamples(const string& file, vector<AST::Var>* vars) {
  std::ifstream input(file);
  vector<Example> output;
  json examples;
  input >> examples;
  bool first = true;
  for (json example : examples) {
    Example new_ex;
    map<string, SymEntry> table;
    for (json input : example) {
      if (input["name"] != "output") {
        vector<int> dim = input["dim"];
        if (first) {
          Var var(input["name"], Dimension(dim.data()),
                  StringToType(input["type"]));
          vars->push_back(var);
        }

        // Add Symbol Table Entries for each variables
        if (StringToType(input["type"]) == NUM) {
          SymEntry entry(input["value"]);
          table[input["name"]] = entry;
        } else if (StringToType(input["type"]) == VEC) {
          vector<float> value_vec = input["value"];
          SymEntry entry(Vector2f(value_vec.data()));
          table[input["name"]] = entry;
        }
      } else {
        new_ex.symbol_table_ = table;
        new_ex.result_ = SymEntry(input["value"]);
      }
    }
    output.push_back(new_ex);
  }
  return output;
}

int main(int argc, char* argv[]) {
  Num five(5, {1, 0, 0});
  Num seven(7, {1, 0, 0});
  Num x(6.1, {1, -1, 0});
  Num y(8.2, {-1, 0, 1});
  Num z(9.3, {1, 0, -1});
  BinOp adder(make_shared<Num>(five), make_shared<Num>(seven), "Plus");
  BinOp upper(make_shared<Num>(seven), make_shared<BinOp>(adder), "Plus");

  // Simple Example
  vector<Var> variables;
  vector<Example> examples =
      ReadExamples("examples/test_labmeeting_july2.json", &variables);
  Var a("A", {1, 0, 0}, NUM);
  Var b("B", {1, 0, 0}, NUM);

  Param hole("hole", {1, 0, 0}, NUM);

  cout << "---- Basic AST Evaluation Example ----" << endl << endl;
  AST::Interp interpreter(examples[0]);

  cout << "---- Variables ----" << endl;
  PrintAST(make_shared<Var>(variables[0]));
  PrintAST(make_shared<Var>(variables[1]));
  PrintAST(make_shared<Var>(variables[2]));

  cout << endl;
  cout << "---- Vars Interpreted ----" << endl;
  PrintAST(variables[0].Accept(&interpreter));
  PrintAST(variables[1].Accept(&interpreter));
  PrintAST(variables[2].Accept(&interpreter));

  cout << endl;

  cout << "---- Vars Added ----" << endl;
  BinOp var_add(make_shared<Var>(a), make_shared<Var>(b), "Plus");
  BinOp vec_add(make_shared<Var>(variables[2]), make_shared<Var>(variables[3]),
                "Plus");
  PrintAST(make_shared<BinOp>(var_add));
  PrintAST(var_add.Accept(&interpreter));
  PrintAST(make_shared<BinOp>(vec_add));
  PrintAST(vec_add.Accept(&interpreter));

  cout << endl;

  cout << "---- Numeric Type (with Dimension) ----" << endl;
  PrintAST(make_shared<Num>(five));
  PrintAST(make_shared<Num>(x));
  PrintAST(make_shared<Num>(seven));

  cout << endl;
  cout << "---- Function Nesting No Interp ----" << endl;
  PrintAST(make_shared<BinOp>(adder));
  PrintAST(make_shared<BinOp>(upper));

  cout << endl;
  cout << "---- Functions Interpreted ----" << endl;

  PrintAST(adder.Accept(&interpreter));
  PrintAST(upper.Accept(&interpreter));

  cout << endl;
  cout << "---- Basic Enumeration Example ----" << endl << endl;

  Num NegTen(-10, {1, 0, 0});
  Num BadZero(1.4, {1, 1, 1});

  // Adding Library Function Definitions
  vector<FunctionEntry> library = ReadLibrary("ops/test_ops.json");

  // Create inputs
  vector<ast_ptr> inputs, roots;
  roots.push_back(make_shared<Var>(a));
  roots.push_back(make_shared<Var>(b));
  roots.push_back(make_shared<Param>(hole));

  vector<vector<float>> signatures;
  vector<ast_ptr> ops = AST::RecEnumerate(roots, inputs, examples, library,
                                          std::stoi(argv[1]), &signatures);
  cout << "----Roots----" << endl;
  for (auto& node : roots) {
    PrintAST(node);
  }
  cout << endl;
  cout << "----Library----" << endl;
  for (auto& func : library) {
    cout << func.op_ << endl;
  }
  cout << endl;
  cout << "----Inputs----" << endl;
  for (auto& input : inputs) {
    PrintAST(input);
  }
  cout << endl;
  cout << "----Result----" << endl;
  for (auto& op : ops) {
    PrintAST(op);
  }
  cout << endl;
  cout << "---- Interp Result ----" << endl;
  for (auto& op : ops) {
    try {
      PrintAST(op->Accept(&interpreter));
    } catch (const invalid_argument& _) {
      // cout << "Cannot interpret programs with holes." << endl;
    }
  }
  cout << endl;
  cout << "---- Number Enumerated ----" << endl;
  cout << ops.size() << endl << endl;

  cout << "---- Solution ----" << endl;
  for (auto& op : ops) {
    string problem = MakeSMTLIBProblem(op, examples);

    context c;
    solver s(c);
    s.from_string(problem.c_str());
    auto result = s.check();

    if (result == z3::sat) {
      PrintAST(op);
      cout << " where" << endl;
      z3::model m = s.get_model();
      for (int i = 0; i < m.size(); ++i) {
        z3::func_decl v = m[i];
        cout << "  " << v.name() << " = " << m.get_const_interp(v) << endl;
      }

      cout << endl
           << "In SMT-LIB terms, the problem" << endl
           << problem << "is ";

      switch (result) {
        case z3::sat:
          cout << "\033[31m";
          break;
        case z3::unknown:
        case z3::unsat:
          cout << "\033[33m";
          break;
      }

      cout << result << "\033[0m"
           << " with model " << endl
           << s.get_model() << endl
           << endl;

      exit(0);
    }
  }
}
