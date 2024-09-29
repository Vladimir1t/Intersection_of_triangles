#include <iostream>
#include <vector>
#include <cstdint>
#include <unordered_map>

namespace Geometry {

    struct Vect {
        double x = 0;
        double y = 0;
        double z = 0;

        Vect operator-(const Vect& other) const {
            return {x - other.x, y - other.y, z - other.z};
        }

        Vect operator+(const Vect& other) const {
            return {x + other.x, y + other.y, z + other.z};
        }

        template <typename T>
        Vect operator*(const T scalar) const {
            return {x * scalar, y * scalar, z * scalar};
        }
        double count_dot(const Vect& v) const { // скалярное произведение 
            return v.x * x + v.y * y + v.z * z;
        }

        Vect cross(const Vect& v) const {
            return { 
                y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x
            };
        }
    };

    struct Triangle {
        Vect a = {0};
        Vect b = {0};
        Vect c = {0};
    };

class Triangle_intersection {

private:

    std::vector<Triangle> triangle_array;  

    std::unordered_map<uint64_t, uint64_t> hash_t;

    Vect count_Vect_product(const Vect& v1, const Vect& v2) { // векторное произведение

        Vect v_new = {};
        v_new.x = v1.y * v2.z - v1.z * v2.y;
        v_new.y = v1.z * v2.x - v1.x * v2.z;
        v_new.z = v1.x * v2.y - v1.y * v2.x;

        return v_new;
    }

    /** @brief 
     * @param ray_origin
     * @param ray_dir
     * @param tr
     * @param out_intersection_point
     * @return 
     */
    bool ray_intersects_triangle(const Vect& ray_origin, const Vect& ray_dir, const Triangle& tr, Vect& out_intersection_point) {

        const double EPSILON = 0.0000001;
        Vect vertex1 = tr.a, vertex2 = tr.b, vertex3 = tr.c;
        Vect edge1 = vertex2 - vertex1;
        Vect edge2 = vertex3 - vertex1;

        Vect H = ray_dir.cross(edge2);
        double a = edge1.count_dot(H);

        if (std::fabs(a) < EPSILON) 
            return false;

        double f = 1 / a;
        Vect S = ray_origin - vertex1;
        double u = f * (S.count_dot(H));

        if (u < 0 || u > 1) 
            return false;

        Vect Q = S.cross(edge1);
        double v = f * ray_dir.count_dot(Q);

        if (v < 0 || u + v > 1)
            return false;
        
        double t = f * edge2.count_dot(Q);

        if (t > EPSILON) {
            out_intersection_point = ray_origin + ray_dir * t;
            return true;
        }
        else 
            return false;
    }

    bool point_in_triangle(const Vect& point, const Triangle& triangle) {

        Vect v1 = triangle.b - triangle.a;
        Vect v2 = triangle.c - triangle.a;
        Vect v3 = point - triangle.a;

        if ((v1.x * (v2.y * v3.z - v2.z * v3.y) - 
             v1.y * (v2.x * v3.z - v2.z * v3.x) + 
             v1.z * (v2.x * v3.z - v2.z * v3.x)) != 0)
            return 0; // not complanar

        double dot00 = v1.count_dot(v1);
        double dot01 = v1.count_dot(v2);
        double dot02 = v1.count_dot(v3);
        double dot11 = v2.count_dot(v2);
        double dot12 = v2.count_dot(v3);

        double inv_denom = 1 / (dot00 * dot11 - dot01 * dot01);
        double u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        double v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

        return (u >= 0) && (v >= 0) && (u + v <= 1);
    } 

public: 

    // Check if two triangles intersect by testing edge intersections
    bool intersects_triangle(const Triangle& t1, const Triangle& t2) {
        Vect intersectionPoint;

        // Test edges of t1 against t2
        if (ray_intersects_triangle(t1.a, t1.b - t1.a, t2, intersectionPoint) ||
            ray_intersects_triangle(t1.b, t1.c - t1.b, t2, intersectionPoint) ||
            ray_intersects_triangle(t1.c, t1.a - t1.c, t2, intersectionPoint)) {
            std::cout << "[ 1 ]\n";
            return true;
        }

        // Test edges of t2 against t1
        if (ray_intersects_triangle(t2.a, t2.b - t2.a, t1, intersectionPoint) ||
            ray_intersects_triangle(t2.b, t2.c - t2.b, t1, intersectionPoint) ||
            ray_intersects_triangle(t2.c, t2.a - t2.c, t1, intersectionPoint)) {
            std::cout << "[ 2 ]\n";
            return true;
        }

        // Check if any vertex of t1 is inside t2 or vice versa
        if (point_in_triangle(t1.a, t2) || point_in_triangle(t1.b, t2) || point_in_triangle(t1.c, t2)) {
            std::cout << "[ 3 ]\n";
            return true;
        }
        if (point_in_triangle(t2.a, t1) || point_in_triangle(t2.b, t1) || point_in_triangle(t2.c, t1)) {
            std::cout << "[ 4 ]\n";
            return true;
        }

        return false;
    }
};
}