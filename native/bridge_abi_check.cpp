#include "LivoxHmiBridge.h"
#include <cstddef>
#include <iostream>
int main() {
    std::cout << "sizeof(LivoxHmiPoint)=" << sizeof(LivoxHmiPoint) << "\n";
    std::cout << "sizeof(LivoxHmiFrameInfo)=" << sizeof(LivoxHmiFrameInfo) << "\n";
    std::cout << "sizeof(LivoxHmiImuSnapshot)=" << sizeof(LivoxHmiImuSnapshot) << "\n";
    if (sizeof(LivoxHmiPoint) != 16) return 1;
    if (offsetof(LivoxHmiPoint, x_m) != 0 || offsetof(LivoxHmiPoint, reflectivity) != 12) return 2;
    if (sizeof(LivoxHmiFrameInfo) != 24) return 3;
    if (sizeof(LivoxHmiImuSnapshot) != 80) return 4;
    return 0;
}
