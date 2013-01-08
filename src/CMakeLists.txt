cmake_minimum_required(VERSION 2.8)
project(src)

find_package(qibuild)

# Create a executable named src
# with the source file: main.cpp
qi_create_bin(src "main.cpp")

# Add a simple test:
enable_testing()
qi_create_test(test_src "test.cpp")
