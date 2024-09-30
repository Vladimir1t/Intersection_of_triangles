#include <iostream>
#include <array>
#include <cmath>
#include <cassert>

#include "intersection_of_triangles.hpp"

void additionalTests() {
    Geometry::Triangle_intersection tr_int;

    // Test 5: No intersection with separated triangles
    Geometry::Triangle t1 = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    Geometry::Triangle t2 = {{2, 2, 0}, {3, 2, 0}, {2, 3, 0}};
    assert( tr_int.intersects_triangle(t1, t2) == false); // Expected: false

    // Test 6: One triangle inside another
    Geometry::Triangle t3 = {{0, 0, 0}, {5, 0, 0}, {0, 5, 0}};
    Geometry::Triangle t4 = {{1, 1, 0}, {2, 2, 0}, {1, 2, 0}};
    assert( tr_int.intersects_triangle(t3, t4) == true); // Expected: true

    // Test 7: Coplanar triangles, no intersection
    Geometry::Triangle t5 = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    Geometry::Triangle t6 = {{1, 1, 0}, {2, 1, 0}, {1, 2, 0}};
    assert( tr_int.intersects_triangle(t5, t6) == false); // Expected: false

    // Test 8: Edge touching vertex
    Geometry::Triangle t7 = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    Geometry::Triangle t8 = {{1, 0, 0}, {1, 1, 0}, {0, 0, 0}};
    assert( tr_int.intersects_triangle(t7, t8) == true); // Expected: true
    
    // Test 9: Triangles in different planes
    Geometry::Triangle t9 =  {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    Geometry::Triangle t10 = {{0, 0, 1}, {1, 0, 1}, {0, 1, 1}};
    assert( tr_int.intersects_triangle(t9, t10) == false); // Expected: false
    
    // Test 10: Triangles intersecting in 3D space
    Geometry::Triangle t11 = {{0,   0,  0}, {1,  0,  0}, {0,  1,   0}};
    Geometry::Triangle t12 = {{0.5, -1, 0}, {0.5, 1, 0}, {-1, 0.5, 0}};
    assert( tr_int.intersects_triangle(t11, t12) == true); // Expected: true

    // my test
    Geometry::Triangle t13 = {{1, 0, 4}, {5, 2, 0}, {0, 5, 0}};
    Geometry::Triangle t14 = {{1, 1, 0}, {2, 2, 0}, {0, 5, 0}};
    assert( tr_int.intersects_triangle(t13, t14) == true); // Expected: false
    
    std::cout << "All additional tests passed!" << std::endl;
}

void runTest(const Geometry::Triangle& t1, const Geometry::Triangle& t2, bool expectedResult, const std::string& testName) {
    Geometry::Triangle_intersection tr_int;

    bool result = tr_int.intersects_triangle(t1, t2);
    if (result == expectedResult) {
        std::cout << "Test " << testName << " passed.\n";
    } else {
        std::cout << "Test " << testName << " failed.\n";
    }
}

int main() {

    Geometry::Triangle_intersection tr_int;

    // Test 1: Triangles intersect
    Geometry::Triangle triangle1 = {{1, 1, 1}, {4, 1, 1}, {2.5, 4, 1}};
    Geometry::Triangle triangle2 = {{0, 0, 0}, {5, 0, 0}, {2.5, 5, 0}};
    runTest(triangle1, triangle2, false, "Intersection Test 1");

    // Test 2: Triangles are far apart, no intersection
    Geometry::Triangle triangle3 = {{10, 10, 10}, {15, 10, 10}, {12.5, 15, 10}};
    Geometry::Triangle triangle4 = {{0, 0, 0}, {5, 0, 0}, {2.5, 5, 0}};
    runTest(triangle3, triangle4, false, "No Intersection Test");

    // Test 3: One triangle completely inside the other
    Geometry::Triangle triangle5 = {{1.5, 1.5, 1}, {3, 1.5, 1}, {2.25, 3, 1}};
    runTest(triangle1, triangle5, true, "Contained Triangle Test");

    // Test 4: Triangles share an edge but do not intersect otherwise
    Geometry::Triangle triangle6 = {{1, 1, 1}, {4, 1, 1}, {2.5, 1, 4}};
    runTest(triangle1, triangle6, true, "Shared Edge Test");

    // Test 5: Triangles share a vertex but do not otherwise intersect
    Geometry::Triangle triangle7 = {{1, 1, 1}, {0, 5, 0}, {-1, 5, 0}};
    runTest(triangle1, triangle7, true, "Shared Vertex Test");

    additionalTests();

    uint64_t number_tr = 0;
    std::cin >> number_tr;

    // Geometry::Triangle tr = {0}; 
    double x1, y1, z1, x2, y2, z2, x3, y3, z3; 
    for (int i = 0; i < number_tr; ++i) {
        std::cin >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3;
        Geometry::Vect v1(x1, y1, z1);
        Geometry::Vect v2(x2, y2, z2);
        Geometry::Vect v3(x3, y3, z3);
        Geometry::Triangle tr(v1, v2, v3);
        tr_int.add_triangle(tr);
    }
    tr_int.intersect_all();

    return 0;
}
