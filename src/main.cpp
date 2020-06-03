#include <iostream>
#include <z3++.h>

#include "lang.hpp"

using std::cout;
using std::endl;
using z3::context;
using z3::expr;
using z3::sat;
using z3::solver;
using z3::unknown;
using z3::unsat;

int main() {

  context c;
  const expr a = c.real_const("a");
  const expr b = c.real_const("b");
  const expr solveme = (((a * a) + (b * b)) == 1);

  const Quantity aq = Quantity(a, 1, -1, 0);
  const Quantity bq = Quantity(b, 1, -1, 0);
  const Quantity result = aq + bq;

  cout << "Here's what printing a quantity looks like: " << result << endl;
  cout << "and here's the underlying expression: " << result.expression()
       << endl;
  cout << "Is this quantity equal to a similar one?: "
       << (result == Quantity(a, 1, -1, 0) + Quantity(b, 1, -1, 0)) << endl;

  solver s(c);
  s.add(solveme);

  // cout << s << endl;
  // cout << s.to_smt2() << endl;

  switch (s.check()) {
  case unsat:
    cout << "invalid" << endl;
    break;
  case sat:
    cout << "valid" << endl;
    break;
  case unknown:
    cout << "unknown" << endl;
    break;
  }

  return EXIT_SUCCESS;
}