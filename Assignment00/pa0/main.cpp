#include<cmath>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<iostream>

int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;
    std::cout << std::sin(30.0 * M_PI / 180.0) << std::endl; 
    // result:
    // 1
    // 0.5
    // 1.41421
    // 3.14159
    // 0.5 (acos(-1) 的值是 π)
    // 0.5

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    std::cout << w << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;
    // vector dot product
    std::cout << "Example of dot product \n";
    std::cout << v.dot(w) << std::endl; // 计算点积
    // vector cross product
    std::cout << "Example of cross product \n";
    std::cout << v.cross(w) << std::endl; // 计算叉积
    // result:
    // Example of output
    // 1
    // 2
    // 3
    // 1
    // 0
    // 0
    // Example of add
    // 2
    // 2
    // 3
    // Example of scalar multiply
    // 3
    // 6
    // 9
    // 2
    // 4
    // 6
    // Example of dot product
    // 1
    // Example of cross product
    // 0
    // 3
    // -2

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    std::cout << j << std::endl;
    // matrix add i + j
    std::cout << "Example of matrix add \n";
    std::cout << i + j << std::endl; // 矩阵相加
    // matrix scalar multiply i * 2.0
    std::cout << "Example of scalar multiply \n";
    std::cout << i * 2.0 << std::endl; // 矩阵标量乘法
    // matrix multiply i * j
    std::cout << "Example of matrix multiply \n";
    std::cout << i * j << std::endl; // 矩阵乘法
    // matrix multiply vector i * v
    std::cout << "Example of matrix and vector multiply \n";
    std::cout << i * v << std::endl; // 矩阵与向量相乘
    // result:
    // Example of output
    // 1 2 3
    // 4 5 6
    // 7 8 9
    // 2 3 1
    // 4 6 5
    // 9 7 8
    // Example of matrix add
    // 3  5  4
    // 8 11 11
    // 16 15 17
    // Example of scalar multiply
    // 2  4  6
    // 8 10 12
    // 14 16 18
    // Example of matrix multiply
    // 37  36  35
    // 82  84  77
    // 127 132 119
    // Example of matrix and vector multiply
    // 14
    // 32
    // 50

    return 0;
}