#include <iostream>
#include <vector>
#include <cstdint>
#include <set>
#include <limits>
#include <algorithm>
#include <cmath>

namespace Geometry {

uint64_t counter = 0;

struct Vect {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double arr[3] = {};

    Vect(double x, double y, double z) : x(x), y(y), z(z) {
        arr[0] = x;
        arr[1] = y;
        arr[2] = z;
    }  
    Vect() = default;

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

    template <typename T>
    Vect operator/(T scalar) const {
        return Vect(x / scalar, y / scalar, z / scalar);
    }

    double count_dot(const Vect& vect) const { 
        return vect.x * x + vect.y * y + vect.z * z;
    }

    Vect normalize() const {
        double length = std::sqrt(x * x + y * y + z * z);

        if (length == 0) 
            return Vect(0, 0, 0);          
            
        return Vect(x / length, y / length, z / length);
    }

    Vect cross(const Vect& vect) const {
        return { 
            y * vect.z - z * vect.y,
            z * vect.x - x * vect.z,
            x * vect.y - y * vect.x
        };
    }
};

struct Triangle {
    Vect a;
    Vect b;
    Vect c;

    uint64_t index;

    Triangle(const Vect& a, const Vect& b, const Vect& c) : a(a), b(b), c(c) {}
};

/** @brief Trinagle_intersection - class with methods of algorithm detecting intersection
 */  
class Triangle_intersection {

private:

    const double epsilon_ = 0.000000001;

    /** @brief ray_intersects_triangle - detect the intersection of a ray(trinagle side) and another triangle 
     *  @param ray_origin vector 
     *  @param ray_dir vector 
     *  @param tr  tringle 
     *  @return 1 - intersect | 0 - don't intersect 
     */
    bool ray_intersects_triangle(const Vect& ray_origin, const Vect& ray_dir, const Triangle& tr) {

        Vect vertex1 = tr.a, vertex2 = tr.b, vertex3 = tr.c;
        Vect edge1 = vertex2 - vertex1;
        Vect edge2 = vertex3 - vertex1;

        Vect H = ray_dir.cross(edge2);
        double a = edge1.count_dot(H);

        if (std::fabs(a) < epsilon_) 
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

        if (t > epsilon_ && t < 1 + epsilon_)   // intersection_point = ray_origin + ray_dir * t 
            return true;

        else 
            return false;
    }

    bool point_in_triangle(const Vect& point, const Triangle& triangle) {

        Vect v1 = triangle.b - triangle.a;
        Vect v2 = triangle.c - triangle.a;
        Vect v3 = point - triangle.a;

        double are_copmplanar = v1.x * (v2.y * v3.z - v2.z * v3.y) - 
                                v1.y * (v2.x * v3.z - v2.z * v3.x) + 
                                v1.z * (v2.x * v3.y - v2.y * v3.x);

        if (std::fabs(are_copmplanar) > epsilon_)
            return false;           

        double dot00 = v1.count_dot(v1);
        double dot01 = v1.count_dot(v2);
        double dot02 = v1.count_dot(v3);
        double dot11 = v2.count_dot(v2);
        double dot12 = v2.count_dot(v3);

        double denom = dot00 * dot11 - dot01 * dot01;
        if (std::fabs(denom) < epsilon_)
            return false;           
        double inv_denom = 1 / denom;

        double u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        double v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

        return (u >= -epsilon_) && (v >= -epsilon_) && (u + v <= 1 + epsilon_);
    }

    Vect normal(const Triangle& tr) {
        return (tr.b - tr.a).cross(tr.c - tr.a).normalize();
    }

    bool are_planes_parallel(const Triangle& t1, const Triangle& t2) {

        Vect norm1 = normal(t1);
        Vect norm2 = normal(t2);
        double dot_product = norm1.count_dot(norm2);

        return (dot_product > 1 - epsilon_ || dot_product < -(1 - epsilon_)); 
    }

    bool are_triangles_coplanar(const Triangle& tr1, const Triangle& tr2) {

        Vect norm = normal(tr1);
        double d1 = norm.count_dot(tr1.a); 
        double d2 = norm.count_dot(tr2.a); 

        return std::abs(d2 - d1) < epsilon_; 
    }

public: 

    std::vector<Triangle>            triangle_array; 
    inline static std::set<uint64_t> set_index; 

    /** @brief add triangle - push a new triangle into vector  
     *  @param tr new trinagle 
     */
    void add_triangle(Triangle& tr) {
        triangle_array.push_back(tr);
        triangle_array.back().index = triangle_array.size() - 1;
        #ifndef NDEBUG
            std::cout << "Index = " << triangle_array.back().index << '\n';
        #endif
    }
    
    #ifndef NDEBUG
    void intersect_all() { 
        for (uint64_t i = 0; i < triangle_array.size(); ++i)
            for (uint64_t j = i + 1; j < triangle_array.size(); ++j) {
                if (intersects_triangle(triangle_array.at(i), triangle_array.at(j)) == true) {
                    std::cout << "Intersect " << triangle_array.at(i).index << " and " <<  triangle_array.at(j).index << std::endl; 
                    set_index.insert(i);
                    set_index.insert(j);
                }
            }
    }
    #endif

    /** @brief intersects_triangle - detect intersection between two triangles 
     *  @param t1 first triangle
     *  @param t2 second triangle 
     */
    bool intersects_triangle(const Triangle& t1, const Triangle& t2) {
        
        // counter++;

        if (are_planes_parallel(t1, t2)) {
            if (!are_triangles_coplanar(t1, t2)) {
                return false; 
            }
        }
        if (ray_intersects_triangle(t1.a, t1.b - t1.a, t2) ||
            ray_intersects_triangle(t1.b, t1.c - t1.b, t2) ||
            ray_intersects_triangle(t1.c, t1.a - t1.c, t2)) {
            #ifndef NDEBUG
                std::cout << "[ 1 ]\n";
            #endif
            return true;
        }
        if (ray_intersects_triangle(t2.a, t2.b - t2.a, t1) ||
            ray_intersects_triangle(t2.b, t2.c - t2.b, t1) ||
            ray_intersects_triangle(t2.c, t2.a - t2.c, t1)) {
            #ifndef NDEBUG
                std::cout << "[ 2 ]\n";
            #endif
            return true;
        }
        if (point_in_triangle(t1.a, t2) || point_in_triangle(t1.b, t2) || point_in_triangle(t1.c, t2)) {
            #ifndef NDEBUG
                std::cout << "[ 3 ]\n";
            #endif
            return true;
        }
        if (point_in_triangle(t2.a, t1) || point_in_triangle(t2.b, t1) || point_in_triangle(t2.c, t1)) {
            #ifndef NDEBUG
                std::cout << "[ 4 ]\n";
            #endif
            return true;
        }

        return false;
    }
};

/** @brief Optimisation - a class with methods of building BVH tree with AABB
 */
class Optimisation {

private:
    /** @brief AABB (axis-aligned bounding box)
    */
    struct AABB {

        Vect min_point, max_point;

        AABB() {
            min_point = Vect( std::numeric_limits<float>::infinity(),  std::numeric_limits<float>::infinity(),  std::numeric_limits<float>::infinity());
            max_point = Vect(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
        }

        AABB(const Vect& min_point, const Vect& max_point) : min_point(min_point), max_point(max_point) {}

        void expand(const Vect& point) {

            min_point.x = std::min(min_point.x, point.x);
            min_point.y = std::min(min_point.y, point.y);
            min_point.z = std::min(min_point.z, point.z);

            max_point.x = std::max(max_point.x, point.x);
            max_point.y = std::max(max_point.y, point.y);
            max_point.z = std::max(max_point.z, point.z);
        }

        AABB merge(const AABB& a, const AABB& b) const {
            Vect min_point(
                std::min(a.min_point.x, b.min_point.x),
                std::min(a.min_point.y, b.min_point.y),
                std::min(a.min_point.z, b.min_point.z)
            );
            Vect max_point(
                std::max(a.max_point.x, b.max_point.x),
                std::max(a.max_point.y, b.max_point.y),
                std::max(a.max_point.z, b.max_point.z)
            );
            return AABB(min_point, max_point);
        }

        double surface_area() const {
            Vect diff = {max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z};
            return 2.0f * (diff.x * diff.y + diff.x * diff.z + diff.y * diff.z);
        }

        /** @brief intersects - detect the intesection between two AABB
         *  @param other another AABB
         */
        bool intersects(const AABB& other) const {
            #ifndef NDEBUG
                std::cout << "Checking intersection between AABBs:\n";
                std::cout << "This AABB: min(" << min_point.x << ", " << min_point.y << ", " << min_point.z
                          << "), max(" << max_point.x << ", " << max_point.y << ", " << max_point.z << ")\n";
                std::cout << "Other AABB: min(" << other.min_point.x << ", " << other.min_point.y << ", " << other.min_point.z
                          << "), max(" << other.max_point.x << ", " << other.max_point.y << ", " << other.max_point.z << ")\n";
            #endif

            if (max_point.x < other.min_point.x || min_point.x > other.max_point.x) 
                return false;
            if (max_point.y < other.min_point.y || min_point.y > other.max_point.y) 
                return false;
            if (max_point.z < other.min_point.z || min_point.z > other.max_point.z) 
                return false;
    
            return true;
        }
    };

public:
    /** @brief node of bounding volume hierarchy (BVH)
     */
    struct BVH_node {
        AABB bounding_box;

        BVH_node* left  = nullptr;
        BVH_node* right = nullptr;

        std::vector<Triangle> triangles;

        BVH_node(const AABB& box) : bounding_box(box) {}
    };

private:
    /** @brief create_bounding_box - create AABB for the triangles 
     */
    AABB create_bounding_box(const std::vector<Triangle>& triangles) {
        
        AABB box = {};
        for (auto tr: triangles) {
             counter++;
            box.expand(tr.a);
            box.expand(tr.b);
            box.expand(tr.c);
        }
        return box;
    }

    size_t find_best_split(std::vector<Triangle>& triangles, const Geometry::Optimisation::AABB& box) {

        double best_cost  = std::numeric_limits<float>::infinity();
        size_t best_split = 0;
        size_t step       = 0;

        if (triangles.size() > 140) 
            step = 70;
        else if (triangles.size() > 20) 
            step = 10;
        else if (triangles.size() > 4) 
            step = 2;
        else 
            step = 1;

        double parent_area = box.surface_area();
        
        int axis = 0;  
        std::sort(triangles.begin(), triangles.end(), [axis](const Triangle& tr1, const Triangle& tr2) {
            double centroid_A = (tr1.a.arr[axis] + tr1.b.arr[axis] + tr1.c.arr[axis]) / 3.0f;
            double centroid_B = (tr2.a.arr[axis] + tr2.b.arr[axis] + tr2.c.arr[axis]) / 3.0f;
            return centroid_A < centroid_B;
        });

        for (size_t i = step; i < triangles.size(); i += step) { 
            std::vector<Triangle> left_triangles(triangles.begin(), triangles.begin() + i);
            std::vector<Triangle> right_triangles(triangles.begin() + i, triangles.end());

            AABB left_box  = create_bounding_box(left_triangles);
            AABB right_box = create_bounding_box(right_triangles);

            double left_area  = left_box.surface_area();
            double right_area = right_box.surface_area();

            double sah_cost = 2.0f + (left_area / parent_area) * left_triangles.size() + 
                                     (right_area / parent_area) * right_triangles.size() +
                                     0.1f * abs(static_cast<int>(left_triangles.size()) - static_cast<int>(right_triangles.size()));

            if (sah_cost < best_cost && !left_triangles.empty() && !right_triangles.empty()) {
                best_cost = sah_cost;
                best_split = i;
            }
        }

        return best_split;
    }

public:
    /** @brief build_BVH - recursively build BVH tree
     *  @param tringles 
     */
    BVH_node* build_BVH(std::vector<Triangle>& triangles) {

        if (triangles.size() == 1) {
            BVH_node* leaf_node = new BVH_node(create_bounding_box(triangles));
            leaf_node->triangles = triangles;
            return leaf_node;
        }
        AABB box = create_bounding_box(triangles);

        size_t best_split = find_best_split(triangles, box);

        if (best_split == 0 || best_split == triangles.size()) {
            BVH_node* leaf_node = new BVH_node(box);
            leaf_node->triangles = triangles;
            return leaf_node;
        }
        
        std::vector<Triangle> left_triangles (triangles.begin(), triangles.begin() + best_split);
        std::vector<Triangle> right_triangles(triangles.begin() + best_split, triangles.end());

        #ifndef NDEBUG
            for (auto tr: left_triangles)
                std::cout << "left ind " << tr.index << '\n';
            for (auto tr: right_triangles)
                std::cout << "right ind " << tr.index << '\n';
        #endif

        BVH_node* node = new BVH_node(box);
        node->triangles = triangles;
        
        node->left  = build_BVH(left_triangles);
        node->right = build_BVH(right_triangles);

        return node;
    }

    /** @brief check_BVH_intersection - detect intersection between leafs or subtrees
     *  @param node1 - right node of a subtree
     *  @param node2 - left node of a subtree
     */
    void check_BVH_intersection(BVH_node* node1, BVH_node* node2) {

    Triangle_intersection tr_int;

    if (node1->left && node1->right) {          
        check_BVH_intersection(node1->left, node1->right);
    }
    if (node2->left && node2->right) {         
        check_BVH_intersection(node2->left, node2->right);
    }

    if (!node1->bounding_box.intersects(node2->bounding_box)) {  // AABB don,t intersect
        #ifndef NDEBUG
            std::cout << "AABB do not intesect\n";
        #endif
        return;
    }

    if ((!node1->left && !node1->right) || (!node2->left && !node2->right)) {  // intersetc triangles, if one of them is a leaf
      
        for (auto& t1 : node1->triangles) {
            for (auto& t2 : node2->triangles) {
                #ifndef NDEBUG
                    std::cout << "tr1: " << t1.index << '\n';
                    std::cout << "tr2: " << t2.index << '\n';
                #endif
                if (tr_int.intersects_triangle(t1, t2)) {
                    #ifndef NDEBUG
                        std::cout << "Intersection between triangle " << t1.index
                                  << " and triangle " << t2.index << std::endl;
                    #endif
                    tr_int.set_index.insert(t1.index);
                    tr_int.set_index.insert(t2.index);
                }
            }
        }
        return;
    }

    if (node1->left && node2->right) {
        check_BVH_intersection(node1->left, node2->right);
    }
    if (node1->right && node2->left) {
        check_BVH_intersection(node1->right, node2->left);
    }
    if (node1->left && node2->left) {
        check_BVH_intersection(node1->left, node2->left);
    }
    if (node1->right && node2->right) {
        check_BVH_intersection(node1->right, node2->right);
    }
}
};
}



