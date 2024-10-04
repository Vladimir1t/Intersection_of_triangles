#include <iostream>
#include <vector>
#include <cstdint>
#include <set>
#include <limits>
#include <algorithm>

namespace Geometry {

    struct Vect {
        double x;
        double y;
        double z;
        double arr[3];

        Vect(double x, double y, double z) : x(x), y(y), z(z) {
            arr[0] = x;
            arr[1] = y;
            arr[2] = z;
        }  

        Vect() {}

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

        double count_dot(const Vect& v) const { 
            return v.x * x + v.y * y + v.z * z;
        }

        Vect normalize() const {
            double length = std::sqrt(x * x + y * y + z * z);

            if (length == 0) 
                return Vect(0, 0, 0);          
            return Vect(x / length, y / length, z / length);
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

    const double epsilon_ = 0.0000000001;

    /** @brief 
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


        if (t > epsilon_ && t < 1 + epsilon_) {  // intersection_point = ray_origin + ray_dir * t 

            return true;
        }
        else 
            return false;
    }

    bool point_in_triangle(const Vect& point, const Triangle& triangle) {

        Vect v1 = triangle.b - triangle.a;
        Vect v2 = triangle.c - triangle.a;
        Vect v3 = point - triangle.a;

        double result = v1.x * (v2.y * v3.z - v2.z * v3.y) - 
                        v1.y * (v2.x * v3.z - v2.z * v3.x) + 
                        v1.z * (v2.x * v3.y - v2.y * v3.x);

        if (std::fabs(result) > epsilon_)
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
        double d2 = norm.count_dot(tr2.a); // Проверяем, лежит ли точка второго треугольника в этой плоскости

        return std::abs(d2 - d1) < epsilon_; 
    }

public: 

    std::vector<Triangle> triangle_array; 
    inline static std::set<uint64_t>    set_index; 

    void add_triangle(Triangle& tr) {
        triangle_array.push_back(tr);
        tr.index = triangle_array.size();
    }
    
    #ifdef NDEBUG
    void intersect_all() { 
         for (uint64_t i = 0; i < triangle_array.size(); ++i)
             for (uint64_t j = i + 1; j < triangle_array.size(); ++j) {
                 if (intersects_triangle(triangle_array.at(i), triangle_array.at(j)) == true) {
                     std::cout << "intersect " << i << " and " << j << std::endl; 
                     set_index.insert(i);
                     set_index.insert(j);
                 }
             }
        for (uint64_t tr_num: set_index)
            std::cout << tr_num << ' '; 
        std::cout << std::endl;
    }
    #endif

    bool intersects_triangle(const Triangle& t1, const Triangle& t2) {

        if (are_planes_parallel(t1, t2)) {
            if (!are_triangles_coplanar(t1, t2)) {
                return false; 
            }
        }
        if (ray_intersects_triangle(t1.a, t1.b - t1.a, t2) ||
            ray_intersects_triangle(t1.b, t1.c - t1.b, t2) ||
            ray_intersects_triangle(t1.c, t1.a - t1.c, t2)) {
            #ifdef Ndebug
                std::cout << "[ 1 ]\n";
            #endif
            return true;
        }
        if (ray_intersects_triangle(t2.a, t2.b - t2.a, t1) ||
            ray_intersects_triangle(t2.b, t2.c - t2.b, t1) ||
            ray_intersects_triangle(t2.c, t2.a - t2.c, t1)) {
            #ifdef Ndebug
                std::cout << "[ 2 ]\n";
            #endif
            return true;
        }
        if (point_in_triangle(t1.a, t2) || point_in_triangle(t1.b, t2) || point_in_triangle(t1.c, t2)) {
            #ifdef Ndebug
                std::cout << "[ 3 ]\n";
            #endif
            return true;
        }
        if (point_in_triangle(t2.a, t1) || point_in_triangle(t2.b, t1) || point_in_triangle(t2.c, t1)) {
            #ifdef Ndebug
                std::cout << "[ 4 ]\n";
            #endif
            return true;
        }

        return false;
    }
};

namespace Optimisation {

    // Структура AABB (axis-aligned bounding box)
    struct AABB {

        Vect min_point, max_point;

        AABB() {
            min_point = Vect( std::numeric_limits<float>::infinity(),  std::numeric_limits<float>::infinity(),  std::numeric_limits<float>::infinity());
            max_point = Vect(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
        }

        AABB(const Vect& minP, const Vect& maxP) : min_point(minP), max_point(maxP) {}

        // Обновление границ AABB на основе новой точки
        void expand(const Vect& point) {

            min_point.x = std::min(min_point.x, point.x);
            min_point.y = std::min(min_point.y, point.y);
            min_point.z = std::min(min_point.z, point.z);

            max_point.x = std::max(max_point.x, point.x);
            max_point.y = std::max(max_point.y, point.y);
            max_point.z = std::max(max_point.z, point.z);
        }

        AABB merge(const AABB& a, const AABB& b) {
            Vect minPoint(
                std::min(a.min_point.x, b.min_point.x),
                std::min(a.min_point.y, b.min_point.y),
                std::min(a.min_point.z, b.min_point.z)
            );
            Vect maxPoint(
                std::max(a.max_point.x, b.max_point.x),
                std::max(a.max_point.y, b.max_point.y),
                std::max(a.max_point.z, b.max_point.z)
            );
            return AABB(minPoint, maxPoint);
        }

        // Проверка пересечения двух AABB
        bool intersects(const AABB& other) const {
            #ifdef NDEBUG
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
    
    struct BVH_node {
        AABB bounding_box;
        BVH_node* left  = nullptr;
        BVH_node* right = nullptr;

        std::vector<Triangle> triangles;

        BVH_node(const AABB& box) : bounding_box(box) {}
    };

    // Функция для создания ограничивающего объёма (AABB) для набора треугольников
    AABB create_bounding_box(const std::vector<Triangle>& triangles) {
        AABB box;
        for (const Triangle& tr : triangles) {
            box.expand(tr.a);
            box.expand(tr.b);
            box.expand(tr.c);
        }
        return box;
    }

    // Рекурсивное построение BVH
    BVH_node* build_BVH(std::vector<Triangle>& triangles, int depth = 0) {

        if (triangles.size() == 1) {
           BVH_node* leaf_node = new BVH_node(create_bounding_box(triangles));
           leaf_node->triangles = triangles;
           return leaf_node;
        }
        // Создаём ограничивающий объём для всех треугольников
        AABB box = create_bounding_box(triangles);

        int axis = depth % 3;

        // Сортируем треугольники по средней координате на выбранной оси
        sort(triangles.begin(), triangles.end(), [axis](const Triangle& tr1, const Triangle& tr2) {
            float centroid_A = (tr1.a.arr[axis] + tr1.b.arr[axis] + tr1.c.arr[axis]) / 3.0f;
            float centroid_B = (tr2.a.arr[axis] + tr2.b.arr[axis] + tr2.c.arr[axis]) / 3.0f;
            return centroid_A < centroid_B;
        });

        size_t mid = triangles.size() / 2;
        std::vector<Triangle> left_triangles(triangles.begin(), triangles.begin() + mid);
        std::vector<Triangle> right_triangles(triangles.begin() + mid, triangles.end());

        BVH_node* node = new BVH_node(box);
        node->left  = build_BVH(left_triangles,  depth + 1);
        node->right = build_BVH(right_triangles, depth + 1);

        node->bounding_box = box.merge(node->left->bounding_box, node->right->bounding_box);

        return node;
    }

    void check_BVH_intersection(BVH_node* node1, BVH_node* node2) {
    Triangle_intersection tr_int;

    // Если это листовые узлы, проверяем треугольники на пересечение
    if (!node1->left && !node1->right && !node2->left && !node2->right) {
        const auto& triangles1 = node1->triangles;
        const auto& triangles2 = node2->triangles;

        for (const auto& t1 : triangles1) {
            for (const auto& t2 : triangles2) {
                // std::cout << "t1: " << t1.a.x << ' ' << t1.a.y << ' ' << t1.a.z << '\n';
                // std::cout << "t2: " << t2.a.x << ' ' << t2.a.y << ' ' << t2.a.z << '\n';
                if (tr_int.intersects_triangle(t1, t2)) {
                    #ifdef NDEBUG
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

    // Если AABB не пересекаются, нет смысла проверять
    if (!node1->bounding_box.intersects(node2->bounding_box)) {
        #ifdef NDEBUG
            std::cout << "AABB do not intesect\n";
        #endif
        return;
    }

    // Если один или оба узла не являются листьями, рекурсивно проверяем их поддеревья
    if (node1->left && node1->right) {
        check_BVH_intersection(node1->left, node1->right);
    }
    if (node2->left && node2->right) {
        check_BVH_intersection(node2->left, node2->right);
    }
    if (!node1->left || !node1->right) {
        check_BVH_intersection(node1, node2->left);
        check_BVH_intersection(node1, node2->right);
    }
    if (!node2->left || !node2->right) {
        check_BVH_intersection(node1->left, node2);
        check_BVH_intersection(node1->right, node2);
    }

    // Проверка пересечения между поддеревьями
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
}
}


