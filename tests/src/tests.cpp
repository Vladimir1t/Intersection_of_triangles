#include <iostream>
#include <fstream>
#include <cstdlib>
#include <set>
#include <string>
#include <functional>
#include <cstdint>
#include <memory>

#include "intersection_of_triangles.hpp"

static bool run_test(const Geometry::Triangle<double>& t1, const Geometry::Triangle<double>& t2, bool expected_result, const std::string& test_name);
static int run_tests();

int main() {

    return run_tests();
}

bool run_test(const Geometry::Triangle<double>& t1, const Geometry::Triangle<double>& t2, bool expected_result, const std::string& test_name) {

    Geometry::Triangle_intersection<double> tr_int;
    bool result = tr_int.intersects_triangle(t1, t2);
    if (result == expected_result) {
        #ifndef NDEBUG
            std::cout << test_name << " passed.\n";
        #endif 
        return true;
    } 
    else {
        std::cout << test_name << " failed.\n";
        return false;
    }
}

bool run_big_test(const std::set<uint64_t> res_ref, const std::string& file_name) {

    Geometry::Triangle_intersection<double> tr_int;

    Geometry::Optimisation<double> opt;

    std::ifstream in_file;
    in_file.open(file_name);
    if (!in_file.is_open()) {
        std::cerr << "File wasn't opened\n";
        std::exit(0);
    }
    uint64_t number_tr = 0;
    if (!(in_file >> number_tr).good())
        return false;

    double x1, y1, z1, x2, y2, z2, x3, y3, z3; 
    for (int i = 0; i < number_tr; ++i) {
        if ((in_file >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3).good()) {
            Geometry::Triangle<double> tr({x1, y1, z1}, {x2, y2, z2}, {x3, y3, z3});
            tr_int.add_triangle(tr);
        }
    }
    in_file.close();

    std::unique_ptr<Geometry::Optimisation<double>::BVH_node> bvh_root = opt.build_BVH(tr_int.triangle_array);
    opt.check_BVH_intersection(bvh_root->left, bvh_root->right, tr_int);

    bool res = (tr_int.set_index == res_ref);

    tr_int.set_index.clear();
    if (res != true) {
        std::cout << "Test failed\n";
        return false;
    }
    else 
        return true;
}

int run_tests() {

    uint64_t       test_counter = 0;
    const uint64_t Test_num     = 18;

    // Test 1: Triangles intersect
    Geometry::Triangle<double> triangle1({1, 1, 1}, {4, 1, 1}, {2.5, 4, 1});
    Geometry::Triangle<double> triangle2({0, 0, 0}, {5, 0, 0}, {2.5, 5, 0});
    test_counter += run_test(triangle1, triangle2, false, "Intersection Test 1");

    // Test 2: Triangles are far apart, no intersection
    Geometry::Triangle<double> triangle3({10, 10, 10}, {15, 10, 10}, {12.5, 15, 10});
    Geometry::Triangle<double> triangle4({0, 0, 0}, {5, 0, 0}, {2.5, 5, 0});
    test_counter += run_test(triangle3, triangle4, false, "No Intersection Test 2");

    // Test 3: One triangle completely inside the other
    Geometry::Triangle<double> triangle5({1.5, 1.5, 1}, {3, 1.5, 1}, {2.25, 3, 1});
    test_counter += run_test(triangle1, triangle5, true, "Contained Triangle Test 3");

    // Test 4: Triangles share an edge but do not intersect otherwise
    Geometry::Triangle<double> triangle6({1, 1, 1}, {4, 1, 1}, {2.5, 1, 4});
    test_counter += run_test(triangle1, triangle6, true, "Shared Edge Test 4");

    // Test 5: Triangles share a vertex but do not otherwise intersect
    Geometry::Triangle<double> triangle7({1, 1, 1}, {0, 5, 0}, {-1, 5, 0});
    test_counter += run_test(triangle1, triangle7, true, "Shared Vertex Test 5");

    // Test 6:
    Geometry::Triangle<double> triangle8({2, 0, 0}, {1, 1, 0}, {4, 1, 0});
    Geometry::Triangle<double> triangle9({3, 1, 0}, {4, 4, 0}, {5, 7, 0});
    test_counter += run_test(triangle8, triangle9, true, "Line Test 6");
    
    // Test 7:
    Geometry::Triangle<double> triangle10({2, 0, 0}, {1, 1, 0}, {4, 1, 0});
    Geometry::Triangle<double> triangle11({2, 0.5, 0}, {2, 0.5, 0}, {2, 0.5, 0});
    test_counter += run_test(triangle10, triangle11, true, "Point Test 7");

    // Test 8:
    Geometry::Triangle<double> triangle12({2, 0, 1}, {3, 8, 3}, {7, 6, 2});
    Geometry::Triangle<double> triangle13({4, 2, 0}, {4, 4, 2}, {4, 6, 4});
    test_counter += run_test(triangle12, triangle13, true, "Cross line Test 8");

    // Test 9:
    Geometry::Triangle<double> triangle14({2, 0, 1}, {3, 8, 3}, {7, 6, 2});
    Geometry::Triangle<double> triangle15({2, 0, 0}, {2, 0, 2}, {4, 6, 4});
    test_counter += run_test(triangle14, triangle15, true, "Cross line Test 9");    

    // Test 10: No intersection with separated triangles
    Geometry::Triangle<double> tr1({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Geometry::Triangle<double> tr2({2, 2, 0}, {3, 2, 0}, {2, 3, 0});
    test_counter += run_test(tr1, tr2, false, "No intersection Test 10");

    // Test 11: One triangle inside another
    Geometry::Triangle<double> tr3({0, 0, 0}, {5, 0, 0}, {0, 5, 0});
    Geometry::Triangle<double> tr4({1, 1, 0}, {2, 2, 0}, {1, 2, 0});
    test_counter += run_test(tr3, tr4, true, "Intersection Test 11");

    // Test 12: Coplanar triangles, no intersection
    Geometry::Triangle<double> tr5({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Geometry::Triangle<double> tr6({1, 1, 0}, {2, 1, 0}, {1, 2, 0});
    test_counter += run_test(tr5, tr6, false, "No intersection Test 12");

    // Test 13: Edge touching vertex
    Geometry::Triangle<double> tr7({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Geometry::Triangle<double> tr8({1, 0, 0}, {1, 1, 0}, {0, 0, 0});
    test_counter += run_test(tr7, tr8, true, "Intersection Test 13");

    // Test 14: Triangles in different planes
    Geometry::Triangle<double> tr9 ({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    Geometry::Triangle<double> tr10({0, 0, 1}, {1, 0, 1}, {0, 1, 1});
    test_counter += run_test(tr9, tr10, false, "No intersection Test 14");
    
    // Test 15: Triangles intersecting in 3D space
    Geometry::Triangle<double> tr11({0,   0,  0}, {1,  0,  0}, {0,  1,   0});
    Geometry::Triangle<double> tr12({0.5, -1, 0}, {0.5, 1, 0}, {-1, 0.5, 0});
    test_counter += run_test(tr11, tr12, true, "Intersection Test 15");

    // Test 16:
    Geometry::Triangle<double> tr13({1, 0, 4}, {5, 2, 0}, {0, 5, 0});
    Geometry::Triangle<double> tr14({1, 1, 0}, {2, 2, 0}, {0, 5, 0});
    test_counter += run_test(tr13, tr14, true, "Intersection Test 16");

    // Test 17:
    std::set<uint64_t> res_ref1 = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    test_counter += run_big_test(res_ref1, "test.txt");

    // Test 18:
    std::set<uint64_t> res_ref2 = {5, 6, 12, 16, 18, 19, 23, 24, 28, 32, 33, 38, 39, 40, 41, 47, 49, 53, 
                                   56, 59, 61, 62, 67, 71, 74, 77, 86, 87, 93, 96, 98};
    test_counter += run_big_test(res_ref2, "test3.txt");

    if (test_counter == Test_num) {
        std::cout << "All tests passed!" << std::endl;
        return 0;
    }
    else 
        return -1;
}


