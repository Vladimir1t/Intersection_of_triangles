#include <iostream>
#include <cstdint>
#include <memory>
#include <set>

#include "intersection_of_triangles.hpp"

/** @name Intersection of triangles
 *  @brief main of a program 'intersection of trinagles'
 *  [in]  number of triangles
 *  [in]  coorinates of each triangle in 3d
 *  [out] indexes of triangles which intersect
 *  @author Vekhov Vladimir
 */
int main() {

    Geometry::Triangle_intersection<double> tr_int;
    Geometry::Optimisation<double> opt;

    uint64_t number_tr = 0;
    
    if (!(std::cin >> number_tr).good())
        return 0;
    if (number_tr <= 0)
        return 0;

    double x1, y1, z1, x2, y2, z2, x3, y3, z3; 
    for (int i = 0; i < number_tr; ++i) {

        if (!(std::cin >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3).good())
            break;
        Geometry::Triangle<double> tr({x1, y1, z1}, {x2, y2, z2}, {x3, y3, z3});
        tr_int.add_triangle(tr);
    }

    #ifndef NDEBUG
        tr_int.intersect_all();
    #endif
    std::unique_ptr<Geometry::Optimisation<double>::BVH_node> bvh_root = opt.build_BVH(tr_int.triangle_array);

    opt.check_BVH_intersection(bvh_root->left, bvh_root->right, tr_int);

    for (uint64_t tr_num: tr_int.set_index)
        std::cout << tr_num << std::endl; 

    return 0;
}
