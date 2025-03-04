#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

/*
* 作业描述
    给定一个点 P=(2,1), 将该点绕原点先逆时针旋转 45◦，再平移 (1,2), 计算出
变换后点的坐标（要求用齐次坐标进行计算）。
*/


#define PI 3.1415926f  // Define PI constant

// Define conversion factor from degrees to radians
#define DEG2RAD  (PI / 180.0f)


int main() {
  Eigen::Vector3f P(2, 1, 1);  // Define a 3D vector P with coordinates (2, 1, 1)
  Eigen::Matrix3f M;           // Define a 3x3 matrix M

  float rad = 45.0f * DEG2RAD;  // Convert 45 degrees to radians

  // Initialize matrix M with rotation and translation
  M << cos(rad), -sin(rad), 1,
        sin(rad), cos(rad), 2,
        0,        0,        1;

  Eigen::Vector3f P2 = M * P;    // Apply transformation matrix M to vector P
  std::cout << P2 << std::endl;  // Output the transformed vector P2
  return 0;
}
