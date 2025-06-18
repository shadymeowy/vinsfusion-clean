#include <Python.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>

#include "python_paths.h"

extern PyObject *PyInit_mymodule(void);
static PyConfig config;

void mymodule_init() {
  PyConfig_InitPythonConfig(&config);
  config.home = PYTHON_HOME;
  config.module_search_paths_set = 1;

  for (const wchar_t **p = PYTHON_PATHS; *p != NULL; p++) {
    PyStatus status = PyWideStringList_Append(&config.module_search_paths, *p);
    if (PyStatus_Exception(status)) {
      Py_ExitStatusException(status);
    }
  }

  PyImport_AppendInittab("mymodule", PyInit_mymodule);

  PyStatus status = Py_InitializeFromConfig(&config);
  if (PyStatus_Exception(status)) {
    Py_ExitStatusException(status);
  }

  PyEval_InitThreads();

  PyRun_SimpleString("import sys; print('Python initialized with PYTHONHOME:', "
                     "sys.executable)");
  PyRun_SimpleString("import sys; print(sys.path)");
  PyRun_SimpleString("import numpy; print(numpy.__version__)");

  PyObject *mod = PyImport_ImportModule("mymodule");
  if (!mod) {
    PyErr_Print();
    exit(1);
  }
}

void mymodule_deinit() {
  Py_Finalize();
}