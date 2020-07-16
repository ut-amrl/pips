#pragma once

#include "ast.hpp"
#include "enumeration.hpp"

#include <string>
#include <vector>

AST::Type StringToType(const std::string& type_string);
std::vector<AST::FunctionEntry> ReadLibrary(const std::string& file);
std::vector<AST::Example> ReadExamples(const std::string& file, std::vector<AST::Var>* vars);
