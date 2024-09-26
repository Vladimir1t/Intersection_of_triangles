#include <iostream>

#include "intersection_of_triangles.hpp"

int main() {

    Triangle_intersection intersection;

    #ifdef Logging
        std::cout << "--- Intersection of triangles ---\n";
        std::cout << "Input the number of triangels\n"
    #endif 

    uint64_t number_tr = 0;
    std::cin >> number_tr;

    #ifdef Logging
        std::cout << "Input " << number_tr << " sets of triangles\n";
    #endif

    double p1_x = 0, p1_y = 0, p1_z = 0, 
           p2_x = 0, p2_y = 0, p2_z = 0,
           p3_x = 0, p3_y = 0, p3_z = 0;

    for (int i = 0; i < number_tr; ++i) {
        std::cin >> p1_x >> p1_y >> p1_z  
                 >> p2_x >> p2_y >> p2_z 
                 >> p3_x >> p3_y >> p3_z;
                 
        intersection.create_triangle(p1_x, p1_y, p1_z, 
                                     p2_x, p2_y, p2_z,
                                     p3_x, p3_y, p3_z);
    }

}