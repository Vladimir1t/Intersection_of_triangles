cmake_minimum_required(VERSION 3.11)
project("Trinagles_intersection")

set(CMAKE_CXX_STANDARD 17)

set(test_srcs
    tests/src/tests.cpp
)

set(srcs
    main.cpp
)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(FLAGS_DEBUG_1   "-g")
set(FLAGS_DEBUG_2   "-D NDEBUG")

add_library(intersection_lib STATIC ${srcs})
target_include_directories(intersection_lib PUBLIC "include")

add_executable(intersection.x main.cpp)
target_link_libraries(intersection.x intersection_lib)

add_library(test_lib STATIC ${test_srcs})
target_include_directories(test_lib PUBLIC "include")

add_executable(test.x ${test_srcs})
target_link_libraries(test.x test_lib)

target_compile_options(test.x PRIVATE "$<$<CONFIG:RELEASE>:${CMAKE_CXX_FLAGS_RELEASE}>" ${FLAGS_DEBUG_1} ${FLAGS_DEBUG_2})
target_compile_options(intersection.x PRIVATE "$<$<CONFIG:RELEASE>:${CMAKE_CXX_FLAGS_RELEASE}>" ${FLAGS_DEBUG_1} ${FLAGS_DEBUG_2})

# cmake -DCMAKE_BUILD_TYPE=Release -S . -B build
# cmake --build build
# ./build/intersection.x
#
# cmake .. -DCMAKE_CXX_INCLUDE_WHAT_YOU_USE=./../../../../include-what-you-use/build/bin/include-what-you-use
# make
