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

// Hash for using an unordered_set with pairs.
struct pair_hash {
	template <class T1, class T2>
	std::size_t operator () (std::pair<T1, T2> const &pair) const
	{
		std::size_t h1 = std::hash<T1>()(pair.first);
		std::size_t h2 = std::hash<T2>()(pair.second);

		return h1 ^ h2;
	}
};
std::vector<AST::Example> ReadExamples(const std::string& file,
    std::unordered_set<AST::Var>& vars,
    std::unordered_set<std::pair<std::string, std::string>, pair_hash>* transitions);
