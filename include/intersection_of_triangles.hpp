#include <iostream>
#include <vector>
#include <cstdint>
#include <unordered_map>

class Triangle_intersection {

private:

    struct Point {
        double x = 0;
        double y = 0;
        double z = 0;
    };

    struct Vector {
        double x = 0;
        double y = 0;
        double z = 0;
    };

    struct Triangle {
        Point    p1;
        Point    p2;
        Point    p3;
    };
    
    std::vector<Triangle> triangle_array;  // ? 

    std::unordered_map<uint64_t, uint64_t> hash_t;

    double count_dot_product(const Vector& v1, const Vector& v2) { // скалярное произведение 

        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

    Vector div_vector(const Vector& v, double num) { // деление вектора на число

        return {v.x / num, v.y / num, v.z / num};
    }

    Vector count_vector_product(const Vector& v1, const Vector& v2) { // векторное произведение

        Vector v_new = {};
        v_new.x = v1.y * v2.z - v1.z * v2.y;
        v_new.y = v1.z * v2.x - v1.x * v2.z;
        v_new.z = v1.x * v2.y - v1.y * v2.x;

        return v_new;
    }

    bool intersect(Triangle tr1, Triangle tr2) {

    }

public: 

    void create_triangle(double p1_x, double p1_y, double p1_z, 
                         double p2_x, double p2_y, double p2_z,
                         double p3_x, double p3_y, double p3_z) {

        triangle_array.push_back({{p1_x, p1_y, p1_z}, {p2_x, p2_y, p2_z}, {p3_x, p3_y, p3_z}});
        
    }

    bool run_intersection() {
        
        bool result = false;
        for (int i = 0; i < triangle_array.size(); ++i) {
            for (int j = i + 1; j < triangle_array.size(); ++j) {
                result = detect_intersection(triangle_array.at(i), triangle_array.at(j));
            }
        }

    }

    void make_plane(Triangle triangle) {
        // plane equation: ax + by + cz + d = 0

        double a = ((triangle.p2.y - triangle.p1.x) * (triangle.p3.z - triangle.p1.z) - (triangle.p2.z - triangle.p1.z) * (triangle.p3.y - triangle.p1.y));

        double b = ((triangle.p2.z - triangle.p1.z) * (triangle.p3.x - triangle.p1.z) - (triangle.p2.x - triangle.p1.x) * (triangle.p3.z - triangle.p1.z));

        double c = ((triangle.p2.x - triangle.p1.x) * (triangle.p3.y - triangle.p1.y) - (triangle.p3.x - triangle.p1.x) * (triangle.p2.z - triangle.p1.z));

        double d = -((triangle.p2.y - triangle.p1.x) * (triangle.p3.z - triangle.p1.z) - (triangle.p2.z - triangle.p1.z) * (triangle.p3.y - triangle.p1.y)) * triangle.p1.x
                   -((triangle.p2.z - triangle.p1.z) * (triangle.p3.x - triangle.p1.z) - (triangle.p2.x - triangle.p1.x) * (triangle.p3.z - triangle.p1.z)) * triangle.p1.y
                   -((triangle.p2.x - triangle.p1.x) * (triangle.p3.y - triangle.p1.y) - (triangle.p3.x - triangle.p1.x) * (triangle.p2.z - triangle.p1.z)) * triangle.p1.z;
    }

    bool detect_intersection(const Triangle& tr1, const Triangle& tr2) {

        Vector U  = {tr1.p2.x - tr1.p1.x, tr1.p2.y - tr1.p1.y, tr1.p2.z - tr1.p1.z};
        Vector V  = {tr1.p3.x - tr1.p1.x, tr1.p3.y - tr1.p1.y, tr1.p3.z - tr1.p1.z};

        Vector S  = {tr2.p2.x - tr2.p1.x, tr2.p2.y - tr2.p1.y, tr2.p2.z - tr2.p1.z};
        Vector T  = {tr2.p3.x - tr2.p1.x, tr2.p3.y - tr2.p1.y, tr2.p3.z - tr2.p1.z};
        Vector AP = {tr2.p1.x - tr1.p1.x, tr2.p1.y - tr1.p1.y, tr2.p1.z - tr1.p1.z};

        //double d = () * ();
        Vector vector_product_U_V = count_vector_product(U, V);

        // d = [U, V] * [U, V]
        double d = count_dot_product(vector_product_U_V, vector_product_U_V);
        std::cout << "coeff d = " << d << '\n';
        
        // a = [S, [U, V]]  / d
        Vector a = div_vector(count_vector_product(S, vector_product_U_V), d);
        std::cout << "vect a: x = " << a.x << "y = " << a.y << "z = " << a.z << '\n';
        // b = [T, [U, V]]  / d
        Vector b = div_vector(count_vector_product(T, vector_product_U_V), d);
        std::cout << "vect b: x = " << b.x << "y = " << b.y << "z = " << b.z << '\n';
        // y = [AP, [U, V]] / d   
        Vector y = div_vector(count_vector_product(AP, vector_product_U_V), d);
        std::cout << "vect y: x = " << y.x << "y = " << y.y << "z = " << y.z << '\n';




        // 1. паралелльны или пересекаются плоскости (a1 == a2, b1 == b2, c1 == c2)
        // 2. если пересекаются, ищем прямую пересечения 
        // 3. треугольники должны пересекать эту прямую (Нахождение пересечения плоскости треугольника с отрезком
        //                                               Проверка: лежит ли точка пересечения на отрезке
        //                                               Проверка: лежит ли точка пересечения внутри треугольника)
        // 4. чередуются ли точки пересечения (если совпадают точки, то ок.)
    }

};