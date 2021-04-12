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

#include "ast.hpp"
#include "visitors/interp_visitor.hpp"
#include "visitors/print_visitor.hpp"

using AST::ast_ptr;
using AST::BOOL;
using AST::Dimension;
using AST::Example;
using AST::FunctionEntry;
using AST::NUM;
using AST::STATE;
using AST::SymEntry;
using AST::Type;
using AST::Var;
using AST::VEC;
using Eigen::Vector2f;
using nlohmann::json;
using std::cout;
using std::endl;
using std::ifstream;
using std::invalid_argument;
using std::map;
using std::pair;
using std::runtime_error;
using std::string;
using std::to_string;
using std::unordered_set;
using std::vector;

class unknown_type_error : public runtime_error {
  using runtime_error::runtime_error;
};

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
    throw unknown_type_error(type_string);
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
      throw unknown_type_error(to_string(type));
  }
}

vector<FunctionEntry> ReadLibrary(const string& file) {
  vector<FunctionEntry> result;
  std::ifstream input(file);
  json library;
  input >> library;
  for (int i = 0; i < library.size(); ++i) {
    // Setup for function parsing.
    json& func = library[i];
    FunctionEntry entry;

    // Get operation name, or fail with descriptive error.
    try {
      entry.op_ = func["op"];
    } catch (nlohmann::detail::type_error) {
      const string error = "op not found or wrong type in JSON object #" +
                           to_string(i) + ": " + to_string(func);
      throw library_parsing_error(error);
    }

    // Get output dimensions, or fail with descriptive error
    try {
      const vector<int> out_dim = func["outputDim"];
      if (out_dim.size() != 3) {
        const string error = "outputDim has length " +
                             to_string(out_dim.size()) +
                             " when it should have length 3 in JSON object #" +
                             to_string(i) + ": " + to_string(func);
        throw library_parsing_error(error);
      }
      entry.output_dim_ = Dimension(out_dim.data());
    } catch (nlohmann::detail::type_error) {
      const string error =
          "outputDim not found or wrong type in JSON object #" + to_string(i) +
          ": " + to_string(func);
      throw library_parsing_error(error);
    }

    // Get output type, or fail with descriptive error
    try {
      const string out_type_str = func["outputType"];
      const Type out_type = StringToType(out_type_str);
      entry.output_type_ = out_type;
    } catch (nlohmann::detail::type_error) {
      const string error =
          "outputType not found or wrong type in JSON object #" + to_string(i) +
          ": " + to_string(func);
      throw library_parsing_error(error);
    } catch (unknown_type_error) {
      const string error = "outputType value is invalid in JSON object #" +
                           to_string(i) + ": " + to_string(func);
      throw library_parsing_error(error);
    }

    // Get input dims, or fail with descriptive error
    for (int j = 0; j < func["inputType"].size(); ++j) {
      try {
        const string in_type_str = func["inputType"][j];
        const Type in_type = StringToType(in_type_str);
        entry.input_types_.push_back(in_type);
      } catch (nlohmann::detail::type_error) {
        const string error =
            "inputType not found or wrong type in JSON object #" +
            to_string(i) + ": " + to_string(func);
        throw library_parsing_error(error);
      } catch (unknown_type_error) {
        const string error = "inputType value is invalid in JSON object #" +
                             to_string(i) + ": " + to_string(func);
        throw library_parsing_error(error);
      }
    }

    // Get input dims or fail with descriptive error
    for (int j = 0; j < func["inputDim"].size(); ++j) {
      try {
        const vector<int> in_dim = func["inputDim"][j];
        if (in_dim.size() != 3) {
          const string error =
              "inputDim has length " + to_string(in_dim.size()) +
              " when it should have length 3 in JSON object #" + to_string(i) +
              ": " + to_string(func);
          throw library_parsing_error(error);
        }
        entry.input_dims_.push_back(Dimension(in_dim.data()));
      } catch (nlohmann::detail::type_error) {
        const string error =
            "inputDim not found or wrong type in JSON object #" + to_string(i) +
            ": " + to_string(func);
        throw library_parsing_error(error);
      }
    }

    // Check that we have the same number of dimensions and types.
    const size_t in_dims_size = entry.input_dims_.size();
    const size_t in_types_size = entry.input_types_.size();
    if (in_dims_size != in_types_size) {
      const string error =
          "type/dimension mismatch with " + to_string(in_dims_size) +
          " dimensions and " + to_string(in_types_size) +
          " types in JSON object #" + to_string(i) + ": " + to_string(func);
      throw library_parsing_error(error);
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
  if (window_size <= 0) {
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
      results.insert(results.end(), examples.begin() + start,
                     examples.begin() + end);
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
vector<Example> ReadExamples(const string& file, unordered_set<AST::Var>& vars,
                             vector<pair<string, string>>* transitions) {
  std::ifstream input(file);
  vector<Example> output;
  json examples;
  input >> examples;
  std::map<pair<string, string>, int> trans_count;
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
    auto trans =
        std::make_pair(example["start"]["value"], example["output"]["value"]);
    trans_count[trans] += 1;
    // transitions->insert(trans);
    output.push_back(new_ex);
    if (example["start"]["value"] == "Halt" &&
        example["output"]["value"] == "Follow") {
      cout << "Example" << endl;
      cout << new_ex.symbol_table_["target"] << endl;
    }
  }

  // TODO(jaholtz) this is a mess, but it works for now.
  vector<pair<int, pair<string, string>>> trans;
  for (auto& entry : trans_count) {
    trans.push_back(std::make_pair(entry.second, entry.first));
  }
  sort(trans.begin(), trans.end());
  reverse(trans.begin(), trans.end());
  cout << "----- Transition Demonstrations -----" << endl;
  for (auto& entry : trans) {
    cout << entry.second.first << "->";
    cout << entry.second.second << " : " << entry.first << endl;
    transitions->push_back(entry.second);
  }
  cout << endl;

  return output;
}

vector<string> FilesInDir(const string& path) {
  vector<string> output;
  tinydir_dir dir;
  tinydir_open(&dir, path.c_str());

  while (dir.has_next) {
    tinydir_file file;
    tinydir_readfile(&dir, &file);

    // printf("%s", file.name);
    if (file.is_dir) {
      // printf("/");
    } else {
      output.push_back(file.name);
    }
    // printf("\n");

    tinydir_next(&dir);
  }

  tinydir_close(&dir);
  return output;
}

ast_ptr LoadJson(const string& file) {
  ifstream input_file;
  input_file.open(file);
  ast_ptr recovered;
  if (input_file.good()) {
    json loaded;
    string line;
    input_file.close();
    input_file.open(file);
    input_file >> loaded;
    recovered = AST::AstFromJson(loaded);
  } else {
    AST::Bool none(false);
    recovered = std::make_shared<AST::Bool>(none);
  }
  return recovered;
}

vector<ast_ptr> LoadSketches(
    const string& dir, const vector<std::pair<string, string>>& branches) {
  vector<ast_ptr> output;
  for (auto& branch : branches) {
    const string file = branch.first + "_" + branch.second + ".json";
    const string filename = dir + file;
    output.push_back(LoadJson(filename));
  }
  return output;
}

vector<ast_ptr> LoadSketches(const string& dir,
                             vector<std::pair<string, string>>* branches) {
  vector<ast_ptr> output;
  const vector<string> files = FilesInDir(dir);
  const string d1 = "_";
  const string d2 = ".json";
  for (const string& file : files) {
    const string filename = dir + file;
    output.push_back(LoadJson(filename));
    std::pair<string, string> branch;
    const auto pos1 = file.find(d1);
    const string start = file.substr(0, pos1);
    // Number of characters to retreive
    const int length = file.length() - 6 - pos1;
    const string end = file.substr(pos1 + 1, length);
    branches->push_back({start, end});
  }
  return output;
}

bool ExistsFile(const string& filename) {
  ifstream infile(filename);
  return infile.good();
}
