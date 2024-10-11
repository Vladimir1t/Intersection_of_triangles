#include <iostream>
#include <array>
#include <cmath>
#include <cassert>

#include "intersection_of_triangles.hpp"

int main() {

    Geometry::Triangle_intersection tr_int;
    Geometry::Optimisation opt;

    uint64_t number_tr = 0;
    std::cin >> number_tr;

    double x1, y1, z1, x2, y2, z2, x3, y3, z3; 
    for (int i = 0; i < number_tr; ++i) {

        std::cin >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3;
        Geometry::Triangle tr({x1, y1, z1}, {x2, y2, z2}, {x3, y3, z3});
        tr_int.add_triangle(tr);
    }

    #ifdef NDEBUG
        tr_int.intersect_all();
    #endif
    Geometry::Optimisation::BVH_node* bvh_root = opt.build_BVH(tr_int.triangle_array);

    opt.check_BVH_intersection(bvh_root->left, bvh_root->right);

    for (uint64_t tr_num: tr_int.set_index)
        std::cout << tr_num << std::endl; 
    
    std::cout << '[' << Geometry::counter << "]\n";

    delete bvh_root;

    return 0;
}
