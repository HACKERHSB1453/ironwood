# SPDX-License-Identifier: GPL-3.0-or-later
# B5: Python bindings. Runs the unittest suite in tests/python against the built `ironwood` module (api/).
if(TARGET ironwood_py)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME python_api
             COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=$<TARGET_FILE_DIR:ironwood_py>"
                     ${Python3_EXECUTABLE} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/python -t ${CMAKE_CURRENT_SOURCE_DIR}/python -v)
    set_tests_properties(python_api PROPERTIES LABELS "python")
endif()
