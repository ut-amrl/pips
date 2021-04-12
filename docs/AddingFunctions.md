Adding functions
================
PIPS allows you to define functions that you can include in your libraries to
be used in synthesis. Currently, adding new functions involves editing PIPS'
source code. We hope to make this more convenient in the future.

Our example command
-------------------
In this tutorial, we'll implement a toy command PlusThree that takes a number
and adds 3 to it.

SMT-LIB implementation
----------------------
Before diving into PIPS, it's a good idea to get your function implemented in
SMT-LIB, the format that Z3 uses for input. You need to come up with some
template that can be filled in to create an SMT-LIB expression that does what
you want.

For unary operators, you need a string _u_ so that _u $INPUT)_ is a valid
SMT-LIB expression that evaluates to your function.

For example, our PlusThree function would have u = "(+ 3 ".

For binary operators, you need a string _u_ so that
_u $LHS $RHS)_ is a valid SMT-LIB expression that evaluates to your function.

For example, a regular Plus function would have u = "(+ ".

It can help to define an SMT-LIB function elsewhere so that you can make a
template in this format. For example, a Double function might have
```
(define-fun double ((x!1 Real)) Real
  (* x!1 x!1))
```
so that you could make a template with u = "(double ".

Implement the function
----------------------
Open library_functions.cpp and add a C++ function for your PIPS function. You
can follow the example of the many functions that are already implemented
there.

Your function will accept ast_ptr objects as inputs. It is up to you to
validate that the ASTs they point to are what you expect. Some helpful macros
for doing this are present in the file:
 - ASSERT_DIM: checks that AST has some specific dimensionality, e.g.
   Vector3i(0,0,0)
 - ASSERT_TYPE: checks that AST has some specific type, e.g. NUM
 - ASSERT_DIMS_EQUAL: checks that two ASTS have the same dimensionality
 - ASSERT_TYPES_EQUAL: checks that two ASTs have the same types

If an ast_ptr you've received is inappropriate for some reason, throw an
std::invalid_argument with a descriptive error message.

Once you're convinced that your ast_ptr's are okay, use dynamic_pointer_cast
to convert them into pointers to objects of the correct type. See ast.hpp for
a list of AST types PIPS supports.

Once your inputs have been cast to the correct type, do whatever computations
you need to do and build an AST object for the result. Return a shared pointer
to this object using make_shared.

Here's an example for PlusThree:
```c++
ast_ptr PlusThree(ast_ptr n)
{
    // Check that ast_ptr is valid.
    ASSERT_TYPE(n, NUM);

    // Convert to Num pointer.
    num_ptr n_cast = dynamic_pointer_cast<Num>(n);

    // Do computation and make an object to return.
    Num result(n_cast->value_ + 3.0, n_cast->dims_);

    // Make a shared pointer and return.
    return make_shared<Num>(result);
}
```

Also, add your function's signature to library_functions.hpp.

Add your function to the interpreter
------------------------------------
Open interp_visitor.cpp. Find either InterpVisitor::UnOp or
InterpVisitor::BinOp depending on what kind of function you're adding and add
a new branch to the big if-statement similar to all the ones that are already
there. For PlusThree, you would add
```c++
} else if (op == "PlusThree") {
    result = PlusThree(input);
} else if (...) {
```

Add your function to the SMT-LIB generator
------------------------------------------
Open tosmtlib_visitor.cpp. Add a new branch to either ToSMTLIB::UnOp or
ToSMTLIB::BinOp that appends the string _u_ you designed in the first part
to unop_smtlib or binop_smtlib. For PlusThree, we would add the following to
ToSMTLIB::UnOp:
```c++
} else if (op == "PlusThree") {
binop_smtlib += "(+ 3 ";
} else if (...) {
```

(Maybe) add your SMT-LIB helper
-------------------------------
If you needed to write some helper code to make an SMT-LIB expression that fit
the template format, you'll need to add that too. Find the function
MakeSMTLIBProblem in synthesis.cpp and add that code to the string problem. It
will now be included in every SMT-LIB problem that PIPS sends to Z3 to solve.