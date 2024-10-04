#include <iostream>

#include "intersection_of_triangles.hpp"

static bool run_test(const Geometry::Triangle& t1, const Geometry::Triangle& t2, bool expected_result, const std::string& test_name);
static void run_tests();

int main() {

    run_tests();
}

bool run_test(const Geometry::Triangle& t1, const Geometry::Triangle& t2, bool expected_result, const std::string& test_name) {
   
    Geometry::Triangle_intersection tr_int;
    bool result = tr_int.intersects_triangle(t1, t2);
    if (result == expected_result) {
        //std::cout << test_name << " passed.\n";
        return true;
    } 
    else {
        std::cout << test_name << " failed.\n";
        return false;
    }
}

void run_tests() {

    Geometry::Triangle_intersection tr_int;
    uint64_t test_counter = 0;
    uint64_t Test_num     = 16;

    // Test 1: Triangles intersect
    Geometry::Triangle triangle1({1, 1, 1}, {4, 1, 1}, {2.5, 4, 1});
    Geometry::Triangle triangle2({0, 0, 0}, {5, 0, 0}, {2.5, 5, 0});
    test_counter += run_test(triangle1, triangle2, false, "Intersection Test 1");

    // Test 2: Triangles are far apart, no intersection
    Geometry::Triangle triangle3({10, 10, 10}, {15, 10, 10}, {12.5, 15, 10});
    Geometry::Triangle triangle4({0, 0, 0}, {5, 0, 0}, {2.5, 5, 0});
    test_counter += run_test(triangle3, triangle4, false, "No Intersection Test 2");

    // Test 3: One triangle completely inside the other
    Geometry::Triangle triangle5({1.5, 1.5, 1}, {3, 1.5, 1}, {2.25, 3, 1});
    test_counter += run_test(triangle1, triangle5, true, "Contained Triangle Test 3");

    // Test 4: Triangles share an edge but do not intersect otherwise
    Geometry::Triangle triangle6({1, 1, 1}, {4, 1, 1}, {2.5, 1, 4});
    test_counter += run_test(triangle1, triangle6, true, "Shared Edge Test 4");

    // Test 5: Triangles share a vertex but do not otherwise intersect
    Geometry::Triangle triangle7({1, 1, 1}, {0, 5, 0}, {-1, 5, 0});
    test_counter += run_test(triangle1, triangle7, true, "Shared Vertex Test 5");

    // Test 6:
    Geometry::Triangle triangle8({2, 0, 0}, {1, 1, 0}, {4, 1, 0});
    Geometry::Triangle triangle9({3, 1, 0}, {4, 4, 0}, {5, 7, 0});
    test_counter += run_test(triangle8, triangle9, true, "Line Test 6");
    
    // Test 7:
    Geometry::Triangle triangle10({2, 0, 0}, {1, 1, 0}, {4, 1, 0});
    Geometry::Triangle triangle11({2, 0.5, 0}, {2, 0.5, 0}, {2, 0.5, 0});
    test_counter += run_test(triangle10, triangle11, true, "Point Test 7");

    // Test 8:
    Geometry::Triangle triangle12({2, 0, 1}, {3, 8, 3}, {7, 6, 2});
    Geometry::Triangle triangle13({4, 2, 0}, {4, 4, 2}, {4, 6, 4});
    test_counter += run_test(triangle12, triangle13, true, "Cross line Test 8");

    // Test 9:
    Geometry::Triangle triangle14({2, 0, 1}, {3, 8, 3}, {7, 6, 2});
    Geometry::Triangle triangle15({2, 0, 0}, {2, 0, 2}, {4, 6, 4});
    test_counter += run_test(triangle14, triangle15, true, "Cross line Test 9");    

    // Test 10: No intersection with separated triangles
    Geometry::Triangle tr1({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Geometry::Triangle tr2({2, 2, 0}, {3, 2, 0}, {2, 3, 0});
    test_counter += run_test(tr1, tr2, false, "No intersection Test 10");

    // Test 11: One triangle inside another
    Geometry::Triangle tr3({0, 0, 0}, {5, 0, 0}, {0, 5, 0});
    Geometry::Triangle tr4({1, 1, 0}, {2, 2, 0}, {1, 2, 0});
    test_counter += run_test(tr3, tr4, true, "Intersection Test 11");

    // Test 12: Coplanar triangles, no intersection
    Geometry::Triangle tr5({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Geometry::Triangle tr6({1, 1, 0}, {2, 1, 0}, {1, 2, 0});
    test_counter += run_test(tr5, tr6, false, "No intersection Test 12");

    // Test 13: Edge touching vertex
    Geometry::Triangle tr7({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Geometry::Triangle tr8({1, 0, 0}, {1, 1, 0}, {0, 0, 0});
    test_counter += run_test(tr7, tr8, true, "Intersection Test 13");

    // Test 14: Triangles in different planes
    Geometry::Triangle tr9 ({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Geometry::Triangle tr10({0, 0, 1}, {1, 0, 1}, {0, 1, 1});
    test_counter += run_test(tr9, tr10, false, "No intersection Test 14");
    
    // Test 15: Triangles intersecting in 3D space
    Geometry::Triangle tr11({0,   0,  0}, {1,  0,  0}, {0,  1,   0});
    Geometry::Triangle tr12({0.5, -1, 0}, {0.5, 1, 0}, {-1, 0.5, 0});
    test_counter += run_test(tr11, tr12, true, "Intersection Test 15");

    // Test 16:
    Geometry::Triangle tr13({1, 0, 4}, {5, 2, 0}, {0, 5, 0});
    Geometry::Triangle tr14({1, 1, 0}, {2, 2, 0}, {0, 5, 0});
    test_counter += run_test(tr13, tr14, true, "Intersection Test 16");

    if (test_counter == Test_num)
        std::cout << "All tests passed!" << std::endl;
}

