#include <iostream>
#include <vector>
#include <cstdint>
#include <set>

namespace Geometry {

    struct Vect {
        double x;
        double y;
        double z;

        Vect(double x, double y, double z) : x(x), y(y), z(z) {}  // Vector constructor

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
        double count_dot(const Vect& v) const {  // скалярное произведение 
            return v.x * x + v.y * y + v.z * z;
        }

        // Метод нормализации вектора
        Vect normalize() const {
            double length = std::sqrt(x * x + y * y + z * z);

            if (length == 0) 
                return Vect(0, 0, 0);          // Избежание деления на 0
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

        Triangle(const Vect& a, const Vect& b, const Vect& c) : a(a), b(b), c(c) {}
    };

/** @brief Trinagle_intersection - class with methods of algorithm detecting intersection
 */  
class Triangle_intersection {

private:

    const double epsilon_ = 0.0000000001;

    std::vector<Triangle> triangle_array_;  

    std::set<uint64_t> set_index_;

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

        //std::cout << "res " << t << '\n';

        if (t > epsilon_ && t < 1 + epsilon_) {  // intersection_point = ray_origin + ray_dir * t 

            //std::cout << "res point: " << point_in_triangle(ray_origin + ray_dir * t, tr) << '\n';
            return true;
        }
        else 
            return false;
    }

    bool point_in_triangle(const Vect& point, const Triangle& triangle) {

        Vect v1 = triangle.b - triangle.a;
        Vect v2 = triangle.c - triangle.a;
        Vect v3 = point - triangle.a;

        // Проверка на компланарность с погрешностью
        double result = v1.x * (v2.y * v3.z - v2.z * v3.y) - 
                        v1.y * (v2.x * v3.z - v2.z * v3.x) + 
                        v1.z * (v2.x * v3.y - v2.y * v3.x);

        if (std::fabs(result) > epsilon_)
            return false;           // Точки не компланарны

        double dot00 = v1.count_dot(v1);
        double dot01 = v1.count_dot(v2);
        double dot02 = v1.count_dot(v3);
        double dot11 = v2.count_dot(v2);
        double dot12 = v2.count_dot(v3);

        double denom = dot00 * dot11 - dot01 * dot01;
        if (std::fabs(denom) < epsilon_)
            return false;            // Треугольник вырожден
        double inv_denom = 1 / denom;

        double u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        double v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

        return (u >= -epsilon_) && (v >= -epsilon_) && (u + v <= 1 + epsilon_);
    }

    // Вспомогательная функция для получения нормали плоскости треугольника
    Vect normal(const Triangle& tr) {
        return (tr.b - tr.a).cross(tr.c - tr.a).normalize();
    }

    // Проверка, параллельны ли две плоскости
    bool are_planes_parallel(const Triangle& t1, const Triangle& t2) {

        Vect norm1 = normal(t1);
        Vect norm2 = normal(t2);
        // Векторы нормалей параллельны, если их скалярное произведение равно 1 или -1
        double dot_product = norm1.count_dot(norm2);

        return (dot_product > 1 - epsilon_ || dot_product < -(1 - epsilon_)); 
    }

    // Проверка, лежат ли треугольники в одной плоскости
    bool are_triangles_coplanar(const Triangle& tr1, const Triangle& tr2) {

        Vect norm = normal(tr1);
        double d1 = norm.count_dot(tr1.a); // Определяем плоскость первого треугольника
        double d2 = norm.count_dot(tr2.a); // Проверяем, лежит ли точка второго треугольника в этой плоскости
        // Если расстояние точки второго треугольника от плоскости первого меньше некоторого порога

        return std::abs(d2 - d1) < epsilon_; // Погрешность для проверки
    }

public: 

    void add_triangle(const Triangle& tr) {
        triangle_array_.push_back(tr);
    }
    
    void intersect_all() {

        for (uint64_t i = 0; i < triangle_array_.size(); ++i)
            for (uint64_t j = i + 1; j < triangle_array_.size(); ++j) {
                if (intersects_triangle(triangle_array_.at(i), triangle_array_.at(j)) == true)
                    std::cout << "intersect " << i << " and " << j << std::endl; 
                    set_index_.insert(i);
                    set_index_.insert(j);
            }
    }

    // Check if two triangles intersect by testing edge intersections
    bool intersects_triangle(const Triangle& t1, const Triangle& t2) {

        // Сначала проверяем, параллельны ли плоскости треугольников
           if (are_planes_parallel(t1, t2)) {
               // Если плоскости параллельны, проверяем, лежат ли они в одной плоскости
               if (!are_triangles_coplanar(t1, t2)) {
                   return false; // Треугольники в разных плоскостях и не могут пересекаться
               }
           }
        //Test edges of t1 against t2
        if (ray_intersects_triangle(t1.a, t1.b - t1.a, t2) ||
            ray_intersects_triangle(t1.b, t1.c - t1.b, t2) ||
            ray_intersects_triangle(t1.c, t1.a - t1.c, t2)) {
            std::cout << "[ 1 ]\n";

            return true;
        }
        // Test edges of t2 against t1
        if (ray_intersects_triangle(t2.a, t2.b - t2.a, t1) ||
            ray_intersects_triangle(t2.b, t2.c - t2.b, t1) ||
            ray_intersects_triangle(t2.c, t2.a - t2.c, t1)) {
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