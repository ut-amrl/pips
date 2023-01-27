// Copyright (c) Jarrett Holtz. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "ast/ast.hpp"
using namespace std;

namespace AST {

vector<ast_ptr> findSimilar(ast_ptr& base, vector<ast_ptr>& lib, vector<ast_ptr>& sketches);

class Perturb : public Visitor {
public:
    vector<ast_ptr> sketches_;
    
    void add_candidate(ast_ptr prog);

    Perturb(ast_ptr& base, vector<ast_ptr>& lib);
    ast_ptr Visit(AST* node);
    ast_ptr Visit(TernOp* node);
    ast_ptr Visit(BinOp* node);
    ast_ptr Visit(UnOp* node);
    ast_ptr Visit(Bool* node);
    ast_ptr Visit(Feature* node);
    ast_ptr Visit(Num* node);
    ast_ptr Visit(Param* node);
    ast_ptr Visit(Var* node);
    ast_ptr Visit(Vec* node);

private:
    ast_ptr base_;
    unordered_set<string> hashes_;
    vector<ast_ptr>& lib_;
};

}  // namespace AST
