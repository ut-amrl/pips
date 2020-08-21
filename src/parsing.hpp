#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "ast.hpp"
#include "enumeration.hpp"

AST::Type StringToType(const std::string& type_string);
std::string TypeToString(AST::Type type);
std::vector<AST::FunctionEntry> ReadLibrary(const std::string& file);
std::vector<AST::Example> ReadExamples(const std::string& file,
                                       std::unordered_set<AST::Var>& vars);
