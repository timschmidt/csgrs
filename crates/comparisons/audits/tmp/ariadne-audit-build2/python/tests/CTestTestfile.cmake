# CMake generated Testfile for 
# Source directory: /tmp/ariadne-audit-src2/python/tests
# Build directory: /tmp/ariadne-audit-build2/python/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_python_import "pytest" "/tmp/ariadne-audit-src2/python/tests/test_import.py")
set_tests_properties(test_python_import PROPERTIES  ENVIRONMENT "PYTHONPATH=/tmp/ariadne-audit-build2/python" LABELS "python" WORKING_DIRECTORY "/tmp/ariadne-audit-src2/python/tests" _BACKTRACE_TRIPLES "/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;28;add_test;/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;0;")
add_test(test_python_numeric "pytest" "/tmp/ariadne-audit-src2/python/tests/test_numeric.py")
set_tests_properties(test_python_numeric PROPERTIES  ENVIRONMENT "PYTHONPATH=/tmp/ariadne-audit-build2/python" LABELS "python" WORKING_DIRECTORY "/tmp/ariadne-audit-src2/python/tests" _BACKTRACE_TRIPLES "/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;28;add_test;/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;0;")
add_test(test_python_linear_algebra "pytest" "/tmp/ariadne-audit-src2/python/tests/test_linear_algebra.py")
set_tests_properties(test_python_linear_algebra PROPERTIES  ENVIRONMENT "PYTHONPATH=/tmp/ariadne-audit-build2/python" LABELS "python" WORKING_DIRECTORY "/tmp/ariadne-audit-src2/python/tests" _BACKTRACE_TRIPLES "/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;28;add_test;/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;0;")
add_test(test_python_function "pytest" "/tmp/ariadne-audit-src2/python/tests/test_function.py")
set_tests_properties(test_python_function PROPERTIES  ENVIRONMENT "PYTHONPATH=/tmp/ariadne-audit-build2/python" LABELS "python" WORKING_DIRECTORY "/tmp/ariadne-audit-src2/python/tests" _BACKTRACE_TRIPLES "/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;28;add_test;/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;0;")
add_test(test_python_calculus "pytest" "/tmp/ariadne-audit-src2/python/tests/test_calculus.py")
set_tests_properties(test_python_calculus PROPERTIES  ENVIRONMENT "PYTHONPATH=/tmp/ariadne-audit-build2/python" LABELS "python" WORKING_DIRECTORY "/tmp/ariadne-audit-src2/python/tests" _BACKTRACE_TRIPLES "/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;28;add_test;/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;0;")
add_test(test_python_geometry "pytest" "/tmp/ariadne-audit-src2/python/tests/test_geometry.py")
set_tests_properties(test_python_geometry PROPERTIES  ENVIRONMENT "PYTHONPATH=/tmp/ariadne-audit-build2/python" LABELS "python" WORKING_DIRECTORY "/tmp/ariadne-audit-src2/python/tests" _BACKTRACE_TRIPLES "/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;28;add_test;/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;0;")
add_test(test_python_solvers "pytest" "/tmp/ariadne-audit-src2/python/tests/test_solvers.py")
set_tests_properties(test_python_solvers PROPERTIES  ENVIRONMENT "PYTHONPATH=/tmp/ariadne-audit-build2/python" LABELS "python" WORKING_DIRECTORY "/tmp/ariadne-audit-src2/python/tests" _BACKTRACE_TRIPLES "/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;28;add_test;/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;0;")
add_test(test_python_symbolic "pytest" "/tmp/ariadne-audit-src2/python/tests/test_symbolic.py")
set_tests_properties(test_python_symbolic PROPERTIES  ENVIRONMENT "PYTHONPATH=/tmp/ariadne-audit-build2/python" LABELS "python" WORKING_DIRECTORY "/tmp/ariadne-audit-src2/python/tests" _BACKTRACE_TRIPLES "/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;28;add_test;/tmp/ariadne-audit-src2/python/tests/CMakeLists.txt;0;")
