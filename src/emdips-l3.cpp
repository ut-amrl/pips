#include <dlfcn.h>
#include <gflags/gflags.h>
#include <z3++.h>

#include <algorithm>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "Python.h"
#include "ast/ast.hpp"
#include "ast/enumeration.hpp"
#include "ast/library_functions.hpp"
#include "ast/parsing.hpp"
#include "ast/synthesis.hpp"
#include "visitors/interp_visitor.hpp"
#include "visitors/print_visitor.hpp"

DEFINE_string(ex_file, "examples/data.json", "Examples file");
DEFINE_string(lib_file, "ops/emdips_test.json", "Operation library file");
DEFINE_string(out_dir, "ref/dipsl3/", "Operation library file");
DEFINE_string(feature_file, "features.txt", "File to write features to");
DEFINE_uint32(feat_depth, 3, "Maximum enumeration depth for features.");
DEFINE_uint32(sketch_depth, 2, "Maximum enumeration depth for sketch.");
DEFINE_uint32(window_size, 3,
              "Size of sliding window to subsample demonstrations with.");
DEFINE_double(target_score, 0.15,
              "What log likelihood should be achieved / what proportion of "
              "examples should be SAT to declare victory?");
DEFINE_bool(write_features, false, "Write all enumerated features to a file");
DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");
DEFINE_bool(debug, true, "Enable Debug Printing");
DEFINE_uint32(batch_size, 8, "Number of sketches to solve in one python call");

using namespace AST;
using namespace std;
using Eigen::Vector2f;
using json = nlohmann::json;
using z3::context;
using z3::solver;

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    if (FLAGS_target_score < 0.0) FLAGS_target_score = 0.0;

    // Set up Python
    // Initialize python support
    Py_Initialize();

    PyRun_SimpleString(
        "import os, sys \n"
        "sys.path.append(os.getcwd() + '/pips/src/optimizer') \n");

    PyObject* pFunc;

    // File name
    PyObject* pName = PyUnicode_FromString((char*)"optimizer");

    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {
        // Function name
        pFunc = PyObject_GetAttrString(pModule, (char*)"run_optimizer_threads");

        if (!(pFunc && PyCallable_Check(pFunc))) {
            if (PyErr_Occurred()) PyErr_Print();
            fprintf(stderr, "Cannot find optimization function\n");
        }
    } else {
        PyErr_Print();
        fprintf(stderr, "Failed to load optimization file");
    }


    // Read in the Examples
    // Also obtains variables that can be used (roots for synthesis)
    // and the possible input->output state pairs.
    unordered_set<Var> variables;
    vector<pair<string, string>> transitions;
    vector<Example> examples =
        ReadExamples(FLAGS_ex_file, variables, &transitions);

    // Sort transitions
    sort(transitions.begin(), transitions.end(),
         [](const pair<string, string>& a,
            const pair<string, string>& b) -> bool {
             if (a.first == b.first) {
                 if (a.first == a.second) return 1;
                 if (b.first == b.second) return -1;
                 return a.second < b.second;
             }
             return a.first < b.first;
         });

    examples = WindowExamples(examples, FLAGS_window_size);

    // Turning variables into roots
    vector<ast_ptr> inputs, roots;
    for (const Var& variable : variables) {
        if (variable.name_ != "goal" && variable.name_ != "free_path" &&
            variable.name_ != "DoorState") {
            roots.push_back(make_shared<Var>(variable));
        }
    }

    vector<float> min_accuracies;
    for (auto& trans : transitions) {
        min_accuracies.push_back(FLAGS_target_score);
    }

    cout << "----Roots----" << endl;
    for (auto& node : roots) {
        cout << node << endl;
    }
    cout << endl;

    cout << "----Transitions----" << endl;
    for (int i = 0; i < transitions.size(); i++) {
        cout << transitions[i].first << "->" << transitions[i].second << endl;
    }
    cout << endl;

    // Loading Library Function Definitions
    vector<FunctionEntry> library = ReadLibrary(FLAGS_lib_file);

    cout << "----Library----" << endl;
    for (auto& func : library) {
        cout << func << endl;
    }
    cout << endl;

    // Enumerate features up to a fixed depth
    vector<Signature> signatures;
    vector<ast_ptr> ops = AST::RecEnumerateLogistic(
        roots, inputs, examples, library, FLAGS_feat_depth, &signatures);

    if (FLAGS_debug) {
        cout << "---- Features Synthesized ----" << endl;
        for (auto& feat : ops) {
            cout << feat << endl;
        }
        cout << endl;
    }


    vector<ast_ptr> all_sketches = EnumerateL3(ops, FLAGS_sketch_depth);
    
    cout << "Num total programs: " << all_sketches.size() << endl;
    // for(ast_ptr each: all_sketches){
    //     cout << each << endl;
    // }

    vector<float> accuracies;
    for(int i = 0; i < transitions.size(); i++){
        accuracies.push_back(FLAGS_target_score);
    }

    EmdipsOutput eo = emdipsL3(examples, transitions, all_sketches, min_accuracies, FLAGS_out_dir, FLAGS_batch_size, pFunc);

    cout << "---- Number of Features Enumerated ----" << endl;
    cout << ops.size() << endl << endl;
    cout << endl;

    // Clean up python
    Py_XDECREF(pFunc);
    Py_DECREF(pModule);
    Py_Finalize();

}
