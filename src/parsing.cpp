#include "parsing.hpp"

#include "ast.hpp"

#include <eigen3/Eigen/Core>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>

using AST::Dimension;
using AST::Example;
using AST::FunctionEntry;
using AST::NUM;
using AST::SymEntry;
using AST::Type;
using AST::Var;
using AST::VEC;
using Eigen::Vector2f;
using nlohmann::json;
using std::map;
using std::string;
using std::vector;

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