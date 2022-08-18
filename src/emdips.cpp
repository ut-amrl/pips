#include <dlfcn.h>
#include <gflags/gflags.h>
#include <z3++.h>

#include <vector>
#include <nlohmann/json.hpp>
#include <fstream>

#include "ast/ast.hpp"
#include "ast/synthesis.hpp"

using nlohmann::json;
using namespace std;
using namespace AST;


// // --------------------------------------------------------------------------------
// // Domain specific setup

// string datafilename = "examples/emdips/out/data.json";
// string pf_filename = "examples/emdips/examples/pf0-";
// string operations_lib = "emdips_test.json";

// string HAToString(int ha){
//   if(ha > 0) return "ACC";
//   if(ha < 0) return "DEC";
//   if(ha == 0) return "CON";
//   return "CON";
// }

// Example jsonToExample(json data, int ha1, int ha2){
//   Example ex;
//   ex.symbol_table_["x"] = SymEntry((float) data["x"]["value"]);
//   ex.symbol_table_["v"] = SymEntry((float) data["v"]["value"]);
//   ex.symbol_table_["decMax"] = SymEntry((float) data["decMax"]["value"]);
//   ex.symbol_table_["target"] = SymEntry((float) data["target"]["value"]);
//   ex.symbol_table_["vMax"] = SymEntry((float) data["vMax"]["value"]);
  
//   ex.start_ = SymEntry(HAToString(ha1));
//   ex.result_ = SymEntry(HAToString(ha2));

//   return ex;
// }

// void defineTransitions(){

//   vector<pair<string,string>> transitions;
//   pair<string, string> p1 ("ACC", "DEC");
//   pair<string, string> p2 ("ACC", "CON");
//   pair<string, string> p3 ("CON", "DEC");
//   transitions.push_back(p1);
//   transitions.push_back(p2);
//   transitions.push_back(p3);
// }

// void defineVariables(){

//   vector<Var> variables;
//   Var x ("x", Dimension(1, 0, 0), NUM);
//   Var v ("v", Dimension(1, -1, 0), NUM);
//   Var decMax ("decMax", Dimension(1, -2, 0), NUM);
//   Var vMax ("vMax", Dimension(1, -1, 0), NUM);
//   Var target ("target", Dimension(1, 0, 0), NUM);

//   variables.insert(x);
//   variables.insert(v);
//   variables.insert(decMax);
//   variables.insert(vMax);
//   variables.insert(target);
// }

// void getExamples(){

//   ifstream datafile (datafilename);
//   stringstream buffer;
//   buffer << datafile.rdbuf();
//   json j;
//   j = json::parse(buffer.str());

//   uint numRobots = 15;
//   vector<Example> allExamples;
//   uint dataIndex = 0;

//   for(uint i=0; i<numRobots; i++){
//     ifstream pf_file (pf_filename + to_string(i) + ".csv");
//     string traj_str;
//     vector<vector<int>> trajs;
//     while (getline(pf_file, traj_str)) {
//       stringstream ss (traj_str);
//       vector<int> traj;
//       while(ss.good()){
//         string pf_ha;
//         getline(ss, pf_ha, ',');
//         traj.push_back(stoi(pf_ha));
//       }
//       trajs.push_back(traj);
//     }
//     dataIndex++;
//     for(uint t=0; t<trajs[0].size()-1; t++){
//       for(vector<int> traj : trajs){
//         Example ex = jsonToExample(j[dataIndex], traj[t], traj[t+1]);
//         allExamples.push_back(ex);
//       }
//       dataIndex++;
//     }
//   }
// }

// // --------------------------------------------------------------------------------
// // General setup

// vector<ast_ptr> variablesToFeatures(vector<Var> variables){
//   vector<ast_ptr> inputs, roots;
//   for (const Var& variable : variables) {
//       roots.push_back(make_shared<Var>(variable));
//   }

//   vector<Signature> signatures;
//   vector<ast_ptr> ops = RecEnumerate(roots, inputs, examples, library, feature_depth, &signatures);
//   return ops;
// }




int main(int argc, char* argv[]) {
//   gflags::ParseCommandLineFlags(&argc, &argv, false);




//   ldipsL3(allExamples, transitions, ops, );


  return 0;
}