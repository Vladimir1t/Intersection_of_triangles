# Triangle Intersection Detection

---

## Overview

This program detects intersections between triangles in 3D space. It includes a set of geometric algorithms that allow testing whether two triangles intersect, based on vector operations, edge crossing tests, and point containment checks. The program takes multiple triangles as input and checks for intersections between each pair of triangles.

### Features:
- Supports detection of intersections between 3D triangles.
- Can handle scenarios where triangles are coplanar, parallel, or intersecting in space.
- Includes a test suite with multiple test cases for various intersection scenarios.

## Algorithm Description

The core algorithm consists of several stages:

1. **Vector Operations:**
   - The program uses custom `Vect` structures to perform vector arithmetic such as cross products, dot products, and normalization. These operations are used to calculate edges and check whether vectors are parallel or perpendicular.

2. **Ray-Triangle Intersection:**
   - The algorithm uses ray-tracing techniques to determine if any of the edges of one triangle intersect with another triangle. A ray is cast from each vertex of the first triangle along each of its edges, and a check is performed to see if this ray intersects the other triangle.

3. **Coplanarity and Parallelism:**
   - If two triangles are in the same plane (coplanar) or have parallel planes, additional tests are performed to determine whether their edges overlap or whether one triangle is entirely within the other.

4. **Edge Testing:**
   - The intersection test evaluates all edges of both triangles. If any edge of one triangle crosses into the other triangle, they are considered intersecting.

5. **Point-in-Triangle Test:**
   - This test checks whether any vertex of one triangle lies inside the other triangle. If a vertex is inside, the triangles are intersecting.

### Class and Functions

- **`Vect`:** 
  Handles 3D vector arithmetic such as subtraction, addition, dot products, cross products, and normalization.

- **`Triangle`:** 
  Represents a triangle defined by three vectors (vertices).

- **`Triangle_intersection`:** 
  - Holds a set of triangles and provides methods to check for intersections between any two triangles.
  - Functions include `ray_intersects_triangle`, `point_in_triangle`, `are_planes_parallel`, and `are_triangles_coplanar`.

## Code Usage

### Compilation

1. **Compiling the main program:**
   To compile the program that checks for triangle intersections from input:
   ```bash
   make all
   ```
   This will generate an executable file named `./intersection.x`.

2. **Running the program:**
   Once compiled, you can run the program and input the number of triangles followed by the coordinates of their vertices.
   ```bash
   ./intersection.x
   ```
   Example input:
   ```
   2
   0 0 0  1 0 0  0 1 0
   0 0 1  1 0 1  0 1 1
   ```

3. **Compiling and running the tests:**
   To run the test suite, compile the test program:
   ```bash
   make test
   ```
   Then run the tests:
   ```bash
   ./test.x
   ```

### Example Program

```cpp
#include <iostream>
#include "intersection_of_triangles.hpp"

int main() {
    Geometry::Triangle_intersection tr_int;
    uint64_t number_tr;
    
    std::cin >> number_tr;
    double x1, y1, z1, x2, y2, z2, x3, y3, z3;
    
    for (int i = 0; i < number_tr; ++i) {
        std::cin >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3;
        Geometry::Triangle tr({x1, y1, z1}, {x2, y2, z2}, {x3, y3, z3});
        tr_int.add_triangle(tr);
    }
    
    tr_int.intersect_all();
    return 0;
}
```

### Example Usage

You can run the program and provide input via standard input. Here's an example of two intersecting triangles:

**Input:**
```
2
0 0 0  1 0 0  0 1 0
0 0 1  1 0 1  0 1 1
```

**Output:**
```
1 2
```

### Example Test

You can add specific test cases using the test framework in `tests.cpp`. Here's an example test that checks for intersection:

```cpp
Geometry::Triangle triangle1({1, 1, 1}, {4, 1, 1}, {2.5, 4, 1});
Geometry::Triangle triangle2({0, 0, 0}, {5, 0, 0}, {2.5, 5, 0});
run_test(triangle1, triangle2, false, "Intersection Test 1");
```

## Directory Structure

```
.
├── include/
│   └── intersection_of_triangles.hpp   # Header file with the algorithm
├── src/
│   └── tests.cpp                       # Test suite
├── Makefile                            # Build instructions
├── README.md                           # Documentation
└── main.cpp                            # Main program
```

## Makefile

The `Makefile` automates the compilation and testing process.

- **Build the main program:**
  ```bash
  make all
  ```

- **Build and run the tests:**
  ```bash
  make test
  ```

- **Clean up object files and binaries:**
  ```bash
  make clear
  ```

## Tests

The program includes a comprehensive test suite. Tests cover the following cases:
- Intersecting triangles.
- Triangles far apart.
- One triangle contained inside another.
- Triangles sharing an edge or vertex.

You can add more test cases by modifying `tests.cpp`.

### Example Test Output:
```
--- All tests passed ---
```

## Оптимизация 


Bounding Volume Hierarchy [BVH]:https://en.wikipedia.org/wiki/Bounding_volume_hierarchy — это иерархическая структура данных, которая используется для ускорения проверки пересечений между объектами в 3D пространстве, такими как треугольники. Основная идея заключается в том, что вместо того, чтобы проверять пересечения между всеми объектами напрямую, можно сгруппировать объекты в более крупные объемы (bounding volumes) и проверять пересечения между этими объемами. Если объёмы не пересекаются, то можно избежать проверки всех объектов, находящихся внутри них.

Ниже приведён пошаговый процесс построения и использования BVH для пересечения треугольников:

1. Определение Bounding Volume
Для каждого треугольника (или объекта) создаём ограничивающий объём (Bounding Volume), который будет ограждать этот объект. В случае треугольников чаще всего используют AABB (Axis-Aligned Bounding Box), так как они просты в вычислении.

2. Построение дерева BVH
Построение дерева BVH — это процесс рекурсивного разбиения набора треугольников на группы и построения ограничивающих объёмов для каждой группы. Это можно сделать несколькими способами, но часто используется подход разделения вдоль осей (аналогично KD-дереву).

3. Определение пересекающихся поддеревьев
Если поддеревья пересекаются, то смотрим пересечение соответсвующих треугольников.