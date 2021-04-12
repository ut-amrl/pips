pipslint
========

pipslint is a program that checks the correctness of your function library
files. It will be built when you build PIPS using the instructions in the
README.

To use pipslint, run
```
pipslint -file somefile.json
```

If your file contains errors, pipslint will halt, display an error message,
and exit with status code 1.

If there are no errors, pipslint will congratulate you and exit with status
code 0.

pipslint will also be able to check example files soon.