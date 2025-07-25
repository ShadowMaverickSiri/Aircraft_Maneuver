# 使用Eigen库进行坐标转换的完整指南

## 概述

在飞机机动仿真程序中，坐标转换是一个关键的计算环节。使用Eigen库可以显著提高坐标转换的精度和效率。本指南将详细介绍如何集成Eigen库来改进坐标转换操作。

## 为什么选择Eigen库？

### 优势
1. **高精度矩阵运算**：Eigen提供了优化的线性代数运算
2. **几何变换支持**：内置旋转矩阵、四元数等几何变换工具
3. **数值稳定性**：优秀的数值计算稳定性
4. **易于使用**：直观的API设计
5. **高性能**：模板元编程优化，编译时优化

### 适用场景
- 精确的地球坐标转换
- 复杂的姿态变换
- 多坐标系之间的转换
- 需要高精度的导航计算

## 安装和配置

### 1. 安装Eigen库

#### Ubuntu/Debian
```bash
sudo apt-get update
sudo apt-get install libeigen3-dev
```

#### CentOS/RHEL
```bash
sudo yum install eigen3-devel
```

#### macOS
```bash
brew install eigen
```

#### Windows (使用vcpkg)
```bash
vcpkg install eigen3
```

### 2. CMake配置

创建或修改`CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.10)
project(Aircraft_Maneuver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找Eigen库
find_package(Eigen3 REQUIRED)

# 源文件
set(SOURCES
    main.cpp
    AircraftModelLibrary.cpp
    FighterJet.cpp
    Maneuvers.cpp
    CoordinateTransform.cpp
)

# 创建可执行文件
add_executable(Aircraft_Maneuver ${SOURCES})

# 链接Eigen库
target_link_libraries(Aircraft_Maneuver Eigen3::Eigen)
```

## 核心实现

### 1. 坐标转换类设计

```cpp
#include <Eigen/Dense>
#include "AircraftModelLibrary.h"

class CoordinateTransform {
public:
    // 地球参数
    static constexpr double EARTH_RADIUS = 6371000.0;
    static constexpr double M_PI = 3.14159265358979323846;

    // 地理坐标 ↔ ECEF坐标转换
    static Eigen::Vector3d geodeticToECEF(const GeoPosition& geodetic);
    static GeoPosition ecefToGeodetic(const Eigen::Vector3d& ecef);
    
    // NUE ↔ ECEF速度转换
    static Eigen::Vector3d nueToECEFVelocity(const Vector3& nueVel, const GeoPosition& position);
    static Vector3 ecefToNUEVelocity(const Eigen::Vector3d& ecefVel, const GeoPosition& position);
    
    // 旋转矩阵
    static Eigen::Matrix3d getECEFToNUERotation(const GeoPosition& position);
    static Eigen::Matrix3d getNUEToECEFRotation(const GeoPosition& position);
    
    // 位置更新
    static GeoPosition updateGeoPositionEigen(const GeoPosition& pos, const Vector3& velocity, double dt);
    
    // 距离和方位角计算
    static double calculateDistance(const GeoPosition& pos1, const GeoPosition& pos2);
    static double calculateBearing(const GeoPosition& from, const GeoPosition& to);

private:
    static double degToRad(double degrees);
    static double radToDeg(double radians);
};
```

### 2. 关键方法实现

#### 地理坐标转ECEF
```cpp
Eigen::Vector3d CoordinateTransform::geodeticToECEF(const GeoPosition& geodetic) {
    double lat = degToRad(geodetic.latitude);
    double lon = degToRad(geodetic.longitude);
    double h = geodetic.altitude;
    
    // WGS84椭球体参数
    const double a = 6378137.0;  // 长半轴
    const double e2 = 0.006694379990141316;  // 第一偏心率平方
    
    double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));  // 卯酉圈曲率半径
    
    Eigen::Vector3d ecef;
    ecef.x() = (N + h) * cos(lat) * cos(lon);
    ecef.y() = (N + h) * cos(lat) * sin(lon);
    ecef.z() = (N * (1 - e2) + h) * sin(lat);
    
    return ecef;
}
```

#### 旋转矩阵计算
```cpp
Eigen::Matrix3d CoordinateTransform::getECEFToNUERotation(const GeoPosition& position) {
    double lat = degToRad(position.latitude);
    double lon = degToRad(position.longitude);
    
    // 构建旋转矩阵：从ECEF到NUE
    Eigen::Matrix3d R;
    R << -sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat),
         cos(lat) * cos(lon),  cos(lat) * sin(lon),  sin(lat),
         -sin(lon),            cos(lon),             0;
    
    return R;
}
```

#### 改进的位置更新
```cpp
GeoPosition CoordinateTransform::updateGeoPositionEigen(const GeoPosition& pos, const Vector3& velocity, double dt) {
    // 1. 将当前位置转换为ECEF坐标
    Eigen::Vector3d ecefPos = geodeticToECEF(pos);
    
    // 2. 将NUE速度转换为ECEF速度
    Eigen::Vector3d ecefVel = nueToECEFVelocity(velocity, pos);
    
    // 3. 在ECEF坐标系下更新位置
    Eigen::Vector3d newEcefPos = ecefPos + ecefVel * dt;
    
    // 4. 将新的ECEF位置转换回地理坐标
    return ecefToGeodetic(newEcefPos);
}
```

## 集成到现有项目

### 1. 修改Aircraft类

在`AircraftModelLibrary.cpp`中：

```cpp
void Aircraft::updateKinematics(double dt) {
    Vector3 a = computeAcceleration();
    
    // 更新速度
    velocity.north += a.north * dt;
    velocity.up += a.up * dt;
    velocity.east += a.east * dt;
    
    // 使用Eigen方法更新位置（替换原来的updateGeoPosition）
    position = CoordinateTransform::updateGeoPositionEigen(position, velocity, dt);
}
```

### 2. 添加新的功能

```cpp
// 计算到目标点的距离
double Aircraft::getDistanceToTarget(const GeoPosition& target) {
    return CoordinateTransform::calculateDistance(position, target);
}

// 计算到目标点的方位角
double Aircraft::getBearingToTarget(const GeoPosition& target) {
    return CoordinateTransform::calculateBearing(position, target);
}

// 计算目标点（给定距离和方位角）
GeoPosition Aircraft::getDestination(double distance, double bearing) {
    return CoordinateTransform::calculateDestination(position, distance, bearing);
}
```

## 使用示例

### 基本坐标转换
```cpp
#include "CoordinateTransform.h"

int main() {
    // 创建地理坐标
    GeoPosition beijing{39.9042, 116.4074, 50.0};
    
    // 转换为ECEF坐标
    Eigen::Vector3d ecef = CoordinateTransform::geodeticToECEF(beijing);
    std::cout << "ECEF: " << ecef.transpose() << std::endl;
    
    // 转换回地理坐标
    GeoPosition backToGeodetic = CoordinateTransform::ecefToGeodetic(ecef);
    std::cout << "Back to geodetic: " << backToGeodetic.latitude 
              << ", " << backToGeodetic.longitude << std::endl;
    
    return 0;
}
```

### 速度向量转换
```cpp
// NUE速度向量
Vector3 nueVel{200.0, 10.0, 0.0};  // 200 m/s 向北，10 m/s 上升

// 转换为ECEF速度
Eigen::Vector3d ecefVel = CoordinateTransform::nueToECEFVelocity(nueVel, beijing);
std::cout << "ECEF velocity: " << ecefVel.transpose() << std::endl;

// 转换回NUE速度
Vector3 backToNue = CoordinateTransform::ecefToNUEVelocity(ecefVel, beijing);
```

### 位置更新
```cpp
// 使用Eigen方法更新位置
GeoPosition newPos = CoordinateTransform::updateGeoPositionEigen(currentPos, velocity, dt);
```

## 性能优化建议

### 1. 预计算旋转矩阵
```cpp
class Aircraft {
private:
    Eigen::Matrix3d rotationMatrix;
    bool rotationMatrixValid;
    
public:
    void updateRotationMatrix() {
        if (!rotationMatrixValid) {
            rotationMatrix = CoordinateTransform::getECEFToNUERotation(position);
            rotationMatrixValid = true;
        }
    }
};
```

### 2. 使用Eigen的Map功能
```cpp
// 将Vector3映射到Eigen向量
Eigen::Map<Eigen::Vector3d> velocityMap(&velocity.north);
Eigen::Vector3d ecefVel = rotationMatrix * velocityMap;
```

### 3. 批量处理
```cpp
// 批量处理多个位置更新
Eigen::MatrixXd positions(3, numAircraft);
Eigen::MatrixXd velocities(3, numAircraft);

// 一次性计算所有ECEF位置
Eigen::MatrixXd ecefPositions = positions + velocities * dt;
```

## 测试和验证

### 1. 精度测试
```cpp
void testPrecision() {
    GeoPosition testPos{45.0, 90.0, 1000.0};
    
    // 测试往返转换精度
    Eigen::Vector3d ecef = CoordinateTransform::geodeticToECEF(testPos);
    GeoPosition back = CoordinateTransform::ecefToGeodetic(ecef);
    
    double latError = std::abs(testPos.latitude - back.latitude);
    double lonError = std::abs(testPos.longitude - back.longitude);
    
    std::cout << "Latitude error: " << latError << " degrees" << std::endl;
    std::cout << "Longitude error: " << lonError << " degrees" << std::endl;
}
```

### 2. 性能测试
```cpp
void testPerformance() {
    const int iterations = 1000000;
    GeoPosition pos{45.0, 90.0, 1000.0};
    Vector3 vel{200.0, 10.0, 0.0};
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < iterations; ++i) {
        CoordinateTransform::updateGeoPositionEigen(pos, vel, 0.1);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "Time for " << iterations << " updates: " << duration.count() << " ms" << std::endl;
}
```

## 常见问题和解决方案

### 1. 编译错误
**问题**：找不到Eigen头文件
**解决**：确保正确安装了Eigen库，并在CMakeLists.txt中正确配置

### 2. 精度问题
**问题**：高纬度地区精度下降
**解决**：使用WGS84椭球体模型，考虑地球扁率

### 3. 性能问题
**问题**：坐标转换速度慢
**解决**：预计算旋转矩阵，使用批量处理

### 4. 内存问题
**问题**：Eigen对象占用过多内存
**解决**：使用Eigen::Map避免不必要的内存分配

## 扩展功能

### 1. 添加更多坐标系
```cpp
// ENU坐标系
static Eigen::Vector3d enuToECEFVelocity(const Vector3& enuVel, const GeoPosition& position);

// NED坐标系
static Eigen::Vector3d nedToECEFVelocity(const Vector3& nedVel, const GeoPosition& position);
```

### 2. 四元数旋转
```cpp
#include <Eigen/Geometry>

static Eigen::Quaterniond getAttitudeQuaternion(double roll, double pitch, double yaw);
static Vector3 rotateVector(const Vector3& vec, const Eigen::Quaterniond& q);
```

### 3. 重力模型
```cpp
static double calculateGravity(const GeoPosition& position);
static Vector3 calculateGravityVector(const GeoPosition& position);
```

## 总结

使用Eigen库进行坐标转换可以显著提高飞机机动仿真程序的精度和性能。通过合理的架构设计和优化，可以充分利用Eigen库的优势，实现高质量的坐标转换功能。

关键要点：
1. 正确安装和配置Eigen库
2. 使用WGS84椭球体模型提高精度
3. 合理设计类结构，避免重复计算
4. 进行充分的测试和验证
5. 根据具体需求进行性能优化 