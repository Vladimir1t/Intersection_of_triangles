cmake_minimum_required(VERSION 3.11)
project("Trinagles_intersection")

set(CMAKE_CXX_STANDARD 17)

set(test_srcs
    src/tests.cpp
)

set(srcs
    main.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-D NDEBUG -std=c++20")

add_library(intersection_lib STATIC ${srcs})
target_include_directories(intersection_lib PUBLIC "include")

add_executable(intersection.x main.cpp)
target_link_libraries(intersection.x intersection_lib)

add_library(test_lib STATIC ${test_srcs})
target_include_directories(test_lib PUBLIC "include")

add_executable(test.x ${test_srcs})
target_link_libraries(test.x test_lib)

# target_compile_options(intersection.x PRIVATE ${CMAKE_CXX_FLAGS})
target_compile_options(test.x PRIVATE ${CMAKE_CXX_FLAGS})

# cmake -DCMAKE_BUILD_TYPE=Release -S . -B build
# cmake --build build
# ./build/intesection.x
# индивидуальная работа с таргетами
# allbraces, allperms - библиотеки
# set(APPS allbraces; allperms)
#
# foreach(APP ${APPS})
#    add_executable(${APP} ${APP}.cpp)
#    target_compile_features(${APP} PRIVATE cxx_std_20)
#    target_link_libraries(${APP} frame)
#    target_link_libraries(${APP} comblib)
# endforeach()