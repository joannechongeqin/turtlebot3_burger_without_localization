#include "test_raytracer.hpp"

// a stupid thing to learn how raytracer works
// in terminal, do:
//   g++ raytracer.cpp
//   ./a.out

int main() {
    std::cout << "testing raytracer" << std::endl;

    ee4308::turtle::RayTracer los;
    ee4308::turtle::V2 vertex = los.init(ee4308::turtle::V2d(0.5, 0.5), ee4308::turtle::V2d(4.5, 7.5));
    std::cout << "root vertex: " << vertex << std::endl;
    while (!los.reached())
    {
        std::cout << "root vertex: " << los.next() << std::endl;
    }

}

