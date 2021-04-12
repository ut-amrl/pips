#include <gflags/gflags.h>

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <vector>

#include "ast/parsing.hpp"

using AST::FunctionEntry;
using nlohmann::json;
using std::cerr;
using std::cout;
using std::endl;
using std::ifstream;
using std::vector;

DEFINE_string(file, "", "Path to file to lint");
DEFINE_string(type, "library", "What type of linting should be run?");

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_file == "") {
    cerr << "Please specify a file to lint!" << endl;
    return EXIT_FAILURE;
  }

  ifstream i(FLAGS_file);

  if (FLAGS_type == "library") {
    try {
      ReadLibrary(FLAGS_file);
      cout << "No errors! Good job!" << endl;
      return EXIT_SUCCESS;
    } catch (const library_parsing_error& e) {
      cerr << e.what() << endl;
      return EXIT_FAILURE;
    }
  } else {
    cerr << "File type " << FLAGS_type << " is not supported right now" << endl;
    return EXIT_FAILURE;
  }
}