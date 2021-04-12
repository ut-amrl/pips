List of implemented functions
=============================

Binary operators
----------------

| Name      | Input types                  | Output type         | Effect        |
| --------- | ---------------------------- | ------------------- | ------------- |
| Plus      | number[a,b,c], number[a,b,c] | number[a,b,c]       | x + y         |
| Plus      | vector[a,b,c], vector[a,b,c] | vector[a,b,c]       | x + y         |
| Minus     | number[a,b,c], number[a,b,c] | number[a,b,c]       | x - y         |
| Minus     | vector[a,b,c], vector[a,b,c] | vector[a,b,c]       | x - y         |
| Times     | number[a,b,c], number[d,e,f] | number[a+d,b+e,c+f] | x * y         |
| Times     | number[a,b,c], number[d,e,f] | number[a+d,b+e,c+f] | x * y         |
| DividedBy | number[a,b,c], number[d,e,f] | number[a-d,b-e,c-f] | x / y         |
| DividedBy | number[a,b,c], number[d,e,f] | number[a-d,b-e,c-f] | x / y         |
| Pow       | number[a,b,c], number[0,0,0] | number[a*x,b*x,c*x] | x^y           |
| Cross     | vector[a,b,c], vector[a,b,c] | vector[a,b,c]       | cross2d(x, y) |
| Dot       | vector[a,b,c], vector[a,b,c] | vector[a,b,c]       | x * y         |
| And       | bool, bool                   | bool                | x ∧ y         |
| Or        | bool, bool                   | bool                | x ∨ y         |
| Eq        | number[a,b,c], number[d,e,f] | bool                | x = y         |
| Gt        | number[a,b,c], number[d,e,f] | bool                | x > y         |
| Lt        | number[a,b,c], number[d,e,f] | bool                | x < y         |
| Gte       | number[a,b,c], number[d,e,f] | bool                | x >= y        |
| Lte       | number[a,b,c], number[d,e,f] | bool                | x <= y        |

Unary operators
---------------

| Name | Input type    | Output type      | Effect           |
| ---- | ------------- | ---------------- | ---------------- |
| Abs  | number[a,b,c] | number[a,b,c]    | abs(x)           |
| Sq   | number[a,b,c] | number[2a,2b,2c] | x^2              |
| Cos  | number[0,0,0] | number[0,0,0]    | cos(x)           |
| Sin  | number[0,0,0] | number[0,0,0]    | sin(x)           |
| VecX | vector[a,b,c] | number[a,b,c]    | first elem of x  |
| VecY | vector[a,b,c] | number[a,b,c]    | second elem of x |
| Not  | bool          | bool             | ~x               |