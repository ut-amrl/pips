#include "parsing.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <unordered_set>
struct tinydir_dir;

class TinyDir {
public:
    TinyDir(const std::string& path);
    ~TinyDir();
    std::string BaseName() const;
private:
    tinydir_dir* dir;
};

#include <tinydir.h>

#include "visitors/interp_visitor.hpp"
#include "visitors/print_visitor.hpp"
#include "ast.hpp"

using AST::ast_ptr;
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
using std::ifstream;
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
      throw invalid_argument("unknown type ");
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
  size_t end = window_size;

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

Example JsonToExample(const json& example) {
  Example new_ex;
  map<string, SymEntry> table;
  for (json input : example) {
    if (input.is_array()) {
      for (auto& obs : input) {
        const Vector2f obstacle(obs["pose"][0], obs["pose"][1]);
        new_ex.obstacles_.push_back(obstacle);
      }
    } else if (input["name"] != "output" && input["name"] != "start") {
      vector<int> dim = input["dim"];
      Var var(input["name"], Dimension(dim.data()),
          StringToType(input["type"]));
      table[input["name"]] = json_to_symentry(input);
    }
  }
  new_ex.symbol_table_ = table;
  new_ex.result_ = json_to_symentry(example["output"]);
  new_ex.start_ = json_to_symentry(example["start"]);
  return new_ex;
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
      if (input.is_array()) {
        for (auto& obs : input) {
          const Vector2f obstacle(obs["pose"][0], obs["pose"][1]);
          new_ex.obstacles_.push_back(obstacle);
        }
      } else if (input["name"] != "output" && input["name"] != "start") {
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

vector<string> FilesInDir(const string& path) {
  vector<string> output;
  tinydir_dir dir;
  tinydir_open(&dir, path.c_str());

  while (dir.has_next) {
    tinydir_file file;
    tinydir_readfile(&dir, &file);

    printf("%s", file.name);
    if (file.is_dir) {
      printf("/");
    } else {
      output.push_back(file.name);
    }
    printf("\n");

    tinydir_next(&dir);
  }

  tinydir_close(&dir);
  return output;
}

ast_ptr LoadJson(const string& file) {
  ifstream input_file;
  input_file.open(file);
  json loaded;
  input_file >> loaded;
  ast_ptr recovered = AST::AstFromJson(loaded);
  return recovered;
}

vector<ast_ptr> LoadSketches(const string& dir,
    vector<std::pair<string, string>>* branches) {
  vector<ast_ptr> output;
  const vector<string> files = FilesInDir(dir);
  const string d1 = "_";
  const string d2 = ".";
  for (const string& file : files) {
    output.push_back(LoadJson(file));
    std::pair<string, string> branch;
    const auto pos1 = file.find(d1);
    const auto pos2 = file.find(d2);
    const string start = file.substr(0, pos1);
    const string end = file.substr(pos1, pos2);
    cout << start << "->" << end << endl;
    branches->push_back({start,end});
  }
  return output;
}
