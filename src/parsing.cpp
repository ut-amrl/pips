#include "parsing.hpp"

#include <eigen3/Eigen/Core>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "ast.hpp"

using AST::BOOL;
using AST::Dimension;
using AST::Example;
using AST::FunctionEntry;
using AST::NUM;
using AST::SymEntry;
using AST::Type;
using AST::Var;
using AST::VEC;
using AST::STATE;
using Eigen::Vector2f;
using nlohmann::json;
using std::invalid_argument;
using std::map;
using std::string;
using std::unordered_set;
using std::vector;
using std::pair;
using std::cout;
using std::endl;

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
  } else if (type_string == "STATE") {
      return Type::STATE;
  } else if (type_string == "BOOL") {
    return Type::BOOL;
  } else {
    throw invalid_argument("unknown type " + type_string);
  }
}

string TypeToString(Type type) {
  switch (type) {
    case Type::BOOL:
      return "BOOL";
    case Type::NODE:
      return "NODE";
    case Type::NUM:
      return "NUM";
    case Type::OP:
      return "OP";
    case Type::VAR:
      return "VAR";
    case Type::VEC:
      return "VEC";
    default:
      throw invalid_argument("unknown type " + type);
  }
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

SymEntry json_to_symentry(const json& item) {
  if (StringToType(item["type"]) == BOOL) {
    SymEntry entry((bool)item["value"]);
    return entry;
  } else if (StringToType(item["type"]) == NUM) {
    SymEntry entry((float)item["value"]);
    return entry;
  } else if (StringToType(item["type"]) == STATE) {
    SymEntry entry((string)item["value"]);
    return entry;
  } else if (StringToType(item["type"]) == VEC) {
    vector<float> value_vec = item["value"];
    SymEntry entry(Vector2f(value_vec.data()));
    return entry;
  } else {
    throw invalid_argument("unsupported type");
  }
}

// Parse json examples file into examples and create
// variables for world inputs.
vector<Example> ReadExamples(const string& file,
                             unordered_set<AST::Var>& vars) {
  std::ifstream input(file);
  vector<Example> output;
  json examples;
  input >> examples;
  // TODO(jaholtz) this is never set to false?
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
          vars.insert(var);
        }
        table[input["name"]] = json_to_symentry(input);
      }
    }
    new_ex.symbol_table_ = table;
    new_ex.result_ = json_to_symentry(example["output"]);
    output.push_back(new_ex);
  }
  return output;
}

// Slides a window over the examples looking for transitions points,
// takes all examples on either side of a transition point
vector<Example> WindowExamples(const vector<Example>& examples,
    const int window_size) {
  if (window_size < 0) {
    return examples;
  }
  vector<Example> results;
  int start = 0;
  int center = window_size / 2;
  int end = window_size;

  while (end < examples.size()) {
    const Example ex = examples[center];
    // We've found a transition point
    if (ex.result_.GetString() != ex.start_.GetString()) {
      results.insert(results.end(),
          examples.begin() + start, examples.begin() + end);
      // start += window_size;
      // center += window_size;
      // end += window_size;
    }
      start++;
      center++;
      end++;
  }
  return results;
}

// This version assumes that the file will contain state transitions
// and is used for ASP synthesis. The transitions in the examples
// will be saved to transitions, and written to the examples.
vector<Example> ReadExamples(const string& file,
                             unordered_set<AST::Var>& vars,
                             unordered_set<pair<string,string>,
                                pair_hash>* transitions) {
  std::ifstream input(file);
  vector<Example> output;
  json examples;
  input >> examples;
  for (json example : examples) {
    Example new_ex;
    map<string, SymEntry> table;
    for (json input : example) {
      if (input["name"] != "output" && input["name"] != "start") {
        vector<int> dim = input["dim"];
        Var var(input["name"], Dimension(dim.data()),
            StringToType(input["type"]));
        vars.insert(var);
        table[input["name"]] = json_to_symentry(input);
      }
    }
    new_ex.symbol_table_ = table;
    new_ex.result_ = json_to_symentry(example["output"]);
    new_ex.start_ = json_to_symentry(example["start"]);
    auto trans = std::make_pair(example["start"]["value"],
                                example["output"]["value"]);
    transitions->insert(trans);
    output.push_back(new_ex);
  }
  return output;
}
