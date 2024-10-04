#include <iostream>
#include <array>
#include <cmath>
#include <cassert>

#include "intersection_of_triangles.hpp"

int main() {

    Geometry::Triangle_intersection tr_int;
    //Geometry::Optimisation opt;

    uint64_t number_tr = 0;
    std::cin >> number_tr;

    double x1, y1, z1, x2, y2, z2, x3, y3, z3; 
    for (int i = 0; i < number_tr; ++i) {
        std::cin >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3;
        
        Geometry::Triangle tr({x1, y1, z1}, {x2, y2, z2}, {x3, y3, z3});
        //std::cout << "push\n";
        tr_int.add_triangle(tr);
    }
    //tr_int.intersect_all();

    Geometry::Optimisation::BVH_node* bvh_root = Geometry::Optimisation::build_BVH(tr_int.triangle_array_);

    check_BVH_intersection(bvh_root->left, bvh_root->right);

    for (uint64_t tr_num: Geometry::set_index)
            std::cout << tr_num << ' '; 
    std::cout << std::endl;

    //tr_int.intersect_all();

    delete bvh_root;

    return 0;
}
