#include <iostream>
#include <vector>
#include <cstdint>
#include <set>
#include <limits>
#include <algorithm>
#include <cmath>
#include <list>

namespace Geometry {

template<class coord_t>
class Vect {     

public:

    coord_t x = 0.0;
    coord_t y = 0.0;
    coord_t z = 0.0;

    Vect(coord_t x, coord_t y, coord_t z) : x(x), y(y), z(z) {}  
    Vect() = default;

    Vect operator-(const Vect& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vect operator+(const Vect& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vect operator*(const coord_t& scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vect operator/(coord_t scalar) const {
        return Vect(x / scalar, y / scalar, z / scalar);
    }

    coord_t count_dot(const Vect& vect) const { 
        return vect.x * x + vect.y * y + vect.z * z;
    }

    Vect normalize() const {
        coord_t length = std::sqrt(x * x + y * y + z * z);

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

template<class coord_t>
class Triangle { 

public:
    Vect<coord_t> a;
    Vect<coord_t> b;
    Vect<coord_t> c;

    uint64_t index;

    Triangle(const Vect<coord_t>& a, const Vect<coord_t>& b, const Vect<coord_t>& c) : a(a), b(b), c(c) {}

    Vect<coord_t> normal() const {
        return (b - a).cross(c - a).normalize();
    }

    bool are_triangles_coplanar(const Triangle& other_tr) const {

        const coord_t epsilon_ = 0.00000001;

        Vect<coord_t> norm = other_tr.normal();
        coord_t d1 = norm.count_dot(a); 
        coord_t d2 = norm.count_dot(other_tr.a); 

        return std::abs(d2 - d1) < epsilon_; 
    }
};

/** @brief Trinagle_intersection - class with methods of algorithm detecting intersection
 */  
template<class coord_t>
class Triangle_intersection {

private:

    const coord_t epsilon_ = 0.00000001;

    /** @brief ray_intersects_triangle - detect the intersection of a ray(trinagle side) and another triangle 
     *  @param ray_origin vector 
     *  @param ray_dir vector 
     *  @param tr  tringle 
     *  @return 1 - intersect | 0 - don't intersect 
     */
    bool ray_intersects_triangle(const Vect<coord_t>& ray_origin, const Vect<coord_t>& ray_dir, const Triangle<coord_t>& tr) const { // static const

        Vect<coord_t> vertex1 = tr.a, vertex2 = tr.b, vertex3 = tr.c;
        Vect<coord_t> edge1 = vertex2 - vertex1;
        Vect<coord_t> edge2 = vertex3 - vertex1;

        Vect<coord_t> H = ray_dir.cross(edge2);
        coord_t a = edge1.count_dot(H);

        if (std::fabs(a) < epsilon_) 
            return false;

        coord_t f = 1 / a;
        Vect<coord_t> S = ray_origin - vertex1;
        coord_t u = f * (S.count_dot(H));

        if (u < 0 || u > 1) 
            return false;

        Vect<coord_t> Q = S.cross(edge1);
        coord_t v = f * ray_dir.count_dot(Q);

        if (v < 0 || u + v > 1)
            return false;
        
        coord_t t = f * edge2.count_dot(Q);

        return (t > epsilon_ && t - epsilon_ < 1);   // intersection_point = ray_origin + ray_dir * t 
    }

    bool point_in_triangle(const Vect<coord_t>& point, const Triangle<coord_t>& triangle) const {

        Vect<coord_t> v1 = triangle.b - triangle.a;
        Vect<coord_t> v2 = triangle.c - triangle.a;
        Vect<coord_t> v3 = point - triangle.a;

        coord_t are_copmplanar = v1.x * (v2.y * v3.z - v2.z * v3.y) - 
                                v1.y * (v2.x * v3.z - v2.z * v3.x) + 
                                v1.z * (v2.x * v3.y - v2.y * v3.x);

        if (std::fabs(are_copmplanar) > epsilon_)
            return false;           

        coord_t dot00 = v1.count_dot(v1);
        coord_t dot01 = v1.count_dot(v2);
        coord_t dot02 = v1.count_dot(v3);
        coord_t dot11 = v2.count_dot(v2);
        coord_t dot12 = v2.count_dot(v3);

        coord_t denom = dot00 * dot11 - dot01 * dot01;
        if (std::fabs(denom) < epsilon_)
            return false;           
        coord_t inv_denom = 1 / denom;

        coord_t u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        coord_t v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

        return (u >= -epsilon_) && (v >= -epsilon_) && (u + v <= 1 + epsilon_);
    }

    static Vect<coord_t> normal(const Triangle<coord_t>& tr) {
        return (tr.b - tr.a).cross(tr.c - tr.a).normalize();
    }

    bool are_planes_parallel(const Triangle<coord_t>& t1, const Triangle<coord_t>& t2) const {

        Vect<coord_t> norm1 = normal(t1);
        Vect<coord_t> norm2 = normal(t2);
        coord_t dot_product = norm1.count_dot(norm2);

        return (dot_product > 1 - epsilon_ || dot_product < -(1 - epsilon_)); 
    }

public: 

    std::vector<Triangle<coord_t>> triangle_array; 
    std::set<uint64_t>    set_index;    

    /** @brief add triangle - push a new triangle into vector  
     *  @param tr new trinagle 
     */
    void add_triangle(Triangle<coord_t>& tr) {
    
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
     *  @param tr1 first triangle
     *  @param tr2 second triangle 
     */
    bool intersects_triangle(const Triangle<coord_t>& tr1, const Triangle<coord_t>& tr2) const {
        
        if (are_planes_parallel(tr1, tr2)) {
            if (!tr1.are_triangles_coplanar(tr2)) {
                return false; 
            }
        }
        if (ray_intersects_triangle(tr1.a, tr1.b - tr1.a, tr2) ||
            ray_intersects_triangle(tr1.b, tr1.c - tr1.b, tr2) ||
            ray_intersects_triangle(tr1.c, tr1.a - tr1.c, tr2)) {
            #ifndef NDEBUG
                std::cout << "[ 1 ]\n";
            #endif
            return true;
        }
        if (ray_intersects_triangle(tr2.a, tr2.b - tr2.a, tr1) ||
            ray_intersects_triangle(tr2.b, tr2.c - tr2.b, tr1) ||
            ray_intersects_triangle(tr2.c, tr2.a - tr2.c, tr1)) {
            #ifndef NDEBUG
                std::cout << "[ 2 ]\n";
            #endif
            return true;
        }
        if (point_in_triangle(tr1.a, tr2) || point_in_triangle(tr1.b, tr2) || point_in_triangle(tr1.c, tr2)) {
            #ifndef NDEBUG
                std::cout << "[ 3 ]\n";
            #endif
            return true;
        }
        if (point_in_triangle(tr2.a, tr1) || point_in_triangle(tr2.b, tr1) || point_in_triangle(tr2.c, tr1)) {
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
template<class coord_t>
class Optimisation {

private:
    /** @brief AABB (axis-aligned bounding box)
    */
    class AABB {

    private:

        Vect<coord_t> min_point, max_point;

    public:

        AABB() {
            min_point = Vect<coord_t>( std::numeric_limits<coord_t>::infinity(),  std::numeric_limits<coord_t>::infinity(),  std::numeric_limits<coord_t>::infinity());
            max_point = Vect<coord_t>(-std::numeric_limits<coord_t>::infinity(), -std::numeric_limits<coord_t>::infinity(), -std::numeric_limits<coord_t>::infinity());
        }

        AABB(const Vect<coord_t>& min_point, const Vect<coord_t>& max_point) : min_point(min_point), max_point(max_point) {}

        void expand(const Vect<coord_t>& point) {

            min_point.x = std::min(min_point.x, point.x);
            min_point.y = std::min(min_point.y, point.y);
            min_point.z = std::min(min_point.z, point.z);

            max_point.x = std::max(max_point.x, point.x);
            max_point.y = std::max(max_point.y, point.y);
            max_point.z = std::max(max_point.z, point.z);
        }

        AABB merge(const AABB& a, const AABB& b) const {
            Vect<coord_t> min_point(
                std::min(a.min_point.x, b.min_point.x),
                std::min(a.min_point.y, b.min_point.y),
                std::min(a.min_point.z, b.min_point.z)
            );
            Vect<coord_t> max_point(
                std::max(a.max_point.x, b.max_point.x),
                std::max(a.max_point.y, b.max_point.y),
                std::max(a.max_point.z, b.max_point.z)
            );
            return AABB(min_point, max_point);
        }

        coord_t surface_area() const {
            Vect<coord_t> diff = {max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z};
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

        std::unique_ptr<BVH_node> left  = nullptr;
        std::unique_ptr<BVH_node> right = nullptr;

        std::vector<Triangle<coord_t>> triangles;

        BVH_node(const AABB& box) : bounding_box(box) {}
    };

private:

    /** @brief create_bounding_box - create AABB for the triangles 
     */
    static AABB create_bounding_box(const std::vector<Triangle<coord_t>>& triangles) {
        
        AABB box = {};
        for (auto& tr: triangles) {
            box.expand(tr.a);
            box.expand(tr.b);
            box.expand(tr.c);
        }
        return box;
    }

    static size_t find_best_split(std::vector<Triangle<coord_t>>& triangles, const Geometry::Optimisation<coord_t>::AABB& box) {

        coord_t best_cost  = std::numeric_limits<coord_t>::infinity();
        size_t best_split = 0;
        size_t step       = 0;

        const size_t BIG_STEP    = 70;
        const size_t MIDDLE_STEP = 10;
        const size_t SMALL_STEP  = 2;

        if (triangles.size() > BIG_STEP * 2) 
            step = BIG_STEP;
        else if (triangles.size() > MIDDLE_STEP * 2) 
            step = MIDDLE_STEP;
        else if (triangles.size() > SMALL_STEP * 2) 
            step = SMALL_STEP;
        else 
            step = 1;

        coord_t parent_area = box.surface_area();
        
        std::sort(triangles.begin(), triangles.end(), [](const Triangle<coord_t>& tr1, const Triangle<coord_t>& tr2) {  // sort by x
            coord_t centroid_A = (tr1.a.x + tr1.b.x + tr1.c.x) / 3.0f;
            coord_t centroid_B = (tr2.a.x + tr2.b.x + tr2.c.x) / 3.0f;
            return centroid_A < centroid_B;
        });

        for (size_t i = step; i < triangles.size(); i += step) { 
            std::vector<Triangle<coord_t>> left_triangles(triangles.begin(), triangles.begin() + i);
            std::vector<Triangle<coord_t>> right_triangles(triangles.begin() + i, triangles.end());

            AABB left_box  = create_bounding_box(left_triangles);
            AABB right_box = create_bounding_box(right_triangles);

            coord_t left_area  = left_box.surface_area();
            coord_t right_area = right_box.surface_area();

            coord_t sah_cost = 2.0f + (left_area / parent_area) * left_triangles.size() + 
                                     (right_area / parent_area) * right_triangles.size() +
                                     0.1f * // abs(static_cast<int>(left_triangles.size()) - static_cast<int>(right_triangles.size()));
                                     (std::max(left_triangles.size(), right_triangles.size()) - std::min(left_triangles.size(), right_triangles.size()));

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
    typename std::unique_ptr<BVH_node> build_BVH(std::vector<Triangle<coord_t>>& triangles) {   

        if (triangles.size() == 1) {

            auto leaf_node = std::make_unique<BVH_node>(create_bounding_box(triangles));
            leaf_node->triangles = triangles;
            
            return leaf_node;
        }
        AABB box = create_bounding_box(triangles);

        size_t best_split = find_best_split(triangles, box);

        if (best_split == 0 || best_split == triangles.size()) {
            auto leaf_node = std::make_unique<BVH_node>(box);
            leaf_node->triangles = triangles;
            
            return leaf_node;
        }
        
        std::vector<Triangle<coord_t>> left_triangles (triangles.begin(), triangles.begin() + best_split);
        std::vector<Triangle<coord_t>> right_triangles(triangles.begin() + best_split, triangles.end());

        #ifndef NDEBUG
            for (auto tr: left_triangles)
                std::cout << "left ind " << tr.index << '\n';
            for (auto tr: right_triangles)
                std::cout << "right ind " << tr.index << '\n';
        #endif

        auto node = std::make_unique<BVH_node>(box);
        node->triangles = triangles;
        
        node->left  = build_BVH(left_triangles);
        node->right = build_BVH(right_triangles);
       
        return node;
    
    }
    /** @brief check_BVH_intersection - detect intersection between leafs or subtrees
     *  @param node1 - right node of a subtree
     *  @param node2 - left node of a subtree
     */
    static void check_BVH_intersection(std::unique_ptr<BVH_node>& node1, std::unique_ptr<BVH_node>& node2, Triangle_intersection<coord_t>& tr_int) {

    if (node1->left && node1->right) {          
        check_BVH_intersection(node1->left, node1->right, tr_int);
    }
    if (node2->left && node2->right) {         
        check_BVH_intersection(node2->left, node2->right, tr_int);
    }

    if (!node1->bounding_box.intersects(node2->bounding_box)) {  
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
        check_BVH_intersection(node1->left, node2->right, tr_int);
    }
    if (node1->right && node2->left) {
        check_BVH_intersection(node1->right, node2->left, tr_int);
    }
    if (node1->left && node2->left) {
        check_BVH_intersection(node1->left, node2->left, tr_int);
    }
    if (node1->right && node2->right) {
        check_BVH_intersection(node1->right, node2->right, tr_int);
    }
}
};
}



