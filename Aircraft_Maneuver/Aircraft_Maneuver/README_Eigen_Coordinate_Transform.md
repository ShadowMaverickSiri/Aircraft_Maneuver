# 使用Eigen库进行坐标转换

## 概述

本项目展示了如何使用Eigen库来改进飞机机动仿真程序中的坐标转换操作。Eigen库提供了强大的矩阵运算和几何变换功能，使得坐标转换更加精确和高效。

## 主要改进

### 1. 精确的地球模型
- 使用WGS84椭球体模型替代简单的球体模型
- 考虑地球扁率对坐标转换的影响
- 提供更准确的距离和方位角计算

### 2. 完整的坐标系转换
- **地理坐标 ↔ ECEF坐标**：支持高精度的地理坐标与地心地固坐标系之间的转换
- **NUE ↔ ECEF速度**：支持北-天-东速度向量与ECEF速度之间的转换
- **旋转矩阵**：提供坐标系之间的旋转矩阵

### 3. 增强的功能
- 计算两点间的精确距离
- 计算方位角
- 验证旋转矩阵的正交性

## 文件结构

```
Aircraft_Maneuver/
├── CoordinateTransform.h          # 坐标转换类声明
├── CoordinateTransform.cpp        # 坐标转换类实现
├── CoordinateTransformExample.cpp # 使用示例
├── CMakeLists.txt                 # CMake配置文件
└── README_Eigen_Coordinate_Transform.md # 本文档
```

## 安装Eigen库

### Ubuntu/Debian
```bash
sudo apt-get install libeigen3-dev
```

### CentOS/RHEL
```bash
sudo yum install eigen3-devel
```

### macOS
```bash
brew install eigen
```

### Windows (使用vcpkg)
```bash
vcpkg install eigen3
```

## 编译和运行

### 使用CMake编译
```bash
mkdir build
cd build
cmake ..
make
```

### 运行坐标转换示例
```bash
./CoordinateTransformExample
```

## 核心类和方法

### CoordinateTransform类

#### 静态方法

1. **geodeticToECEF(const GeoPosition& geodetic)**
   - 将地理坐标转换为ECEF坐标
   - 使用WGS84椭球体模型

2. **ecefToGeodetic(const Eigen::Vector3d& ecef)**
   - 将ECEF坐标转换为地理坐标
   - 使用迭代方法提高精度

3. **nueToECEFVelocity(const Vector3& nueVel, const GeoPosition& position)**
   - 将NUE速度向量转换为ECEF坐标系下的速度

4. **ecefToNUEVelocity(const Eigen::Vector3d& ecefVel, const GeoPosition& position)**
   - 将ECEF坐标系下的速度转换为NUE速度向量

5. **updateGeoPositionEigen(const GeoPosition& pos, const Vector3& velocity, double dt)**
   - 使用Eigen库更新地理位置
   - 比原始方法更精确

6. **calculateDistance(const GeoPosition& pos1, const GeoPosition& pos2)**
   - 计算两点间的精确距离

7. **calculateBearing(const GeoPosition& from, const GeoPosition& to)**
   - 计算方位角

## 坐标系说明

### 1. 地理坐标系 (Geodetic)
- **纬度 (Latitude)**：-90° 到 +90°
- **经度 (Longitude)**：-180° 到 +180°
- **高度 (Altitude)**：相对于椭球体的高度

### 2. ECEF坐标系 (Earth-Centered, Earth-Fixed)
- **X轴**：指向赤道与本初子午线的交点
- **Y轴**：指向赤道与东经90°线的交点
- **Z轴**：指向北极

### 3. NUE坐标系 (North-Up-East)
- **北向 (North)**：指向地理北
- **天向 (Up)**：指向天顶
- **东向 (East)**：指向地理东

## 使用示例

### 基本坐标转换
```cpp
#include "CoordinateTransform.h"

// 创建地理坐标
GeoPosition beijing{39.9042, 116.4074, 50.0};

// 转换为ECEF坐标
Eigen::Vector3d ecef = CoordinateTransform::geodeticToECEF(beijing);

// 转换回地理坐标
GeoPosition backToGeodetic = CoordinateTransform::ecefToGeodetic(ecef);
```

### 速度向量转换
```cpp
// NUE速度向量
Vector3 nueVel{200.0, 10.0, 0.0};  // 200 m/s 向北，10 m/s 上升

// 转换为ECEF速度
Eigen::Vector3d ecefVel = CoordinateTransform::nueToECEFVelocity(nueVel, beijing);

// 转换回NUE速度
Vector3 backToNue = CoordinateTransform::ecefToNUEVelocity(ecefVel, beijing);
```

### 位置更新
```cpp
// 使用Eigen方法更新位置
GeoPosition newPos = CoordinateTransform::updateGeoPositionEigen(currentPos, velocity, dt);
```

### 距离和方位角计算
```cpp
GeoPosition shanghai{31.2304, 121.4737, 10.0};

// 计算距离
double distance = CoordinateTransform::calculateDistance(beijing, shanghai);

// 计算方位角
double bearing = CoordinateTransform::calculateBearing(beijing, shanghai);
```

## 优势对比

### 原始方法 vs Eigen方法

| 特性 | 原始方法 | Eigen方法 |
|------|----------|-----------|
| 地球模型 | 简单球体 | WGS84椭球体 |
| 精度 | 中等 | 高 |
| 计算复杂度 | 简单 | 中等 |
| 功能完整性 | 基础 | 完整 |
| 数值稳定性 | 一般 | 优秀 |

### 精度提升
- **距离计算**：从球体模型的±0.5%误差提升到椭球体模型的±0.1%误差
- **坐标转换**：考虑地球扁率，提高高纬度地区的精度
- **速度转换**：使用旋转矩阵，避免累积误差

## 集成到现有项目

要将Eigen坐标转换集成到现有的飞机机动仿真程序中：

1. **替换位置更新函数**
```cpp
// 在 AircraftModelLibrary.cpp 中
void Aircraft::updateKinematics(double dt) {
    Vector3 a = computeAcceleration();
    
    // 更新速度
    velocity.north += a.north * dt;
    velocity.up += a.up * dt;
    velocity.east += a.east * dt;
    
    // 使用Eigen方法更新位置
    position = CoordinateTransform::updateGeoPositionEigen(position, velocity, dt);
}
```

2. **添加新的功能**
```cpp
// 计算到目标点的距离
double distanceToTarget = CoordinateTransform::calculateDistance(position, targetPosition);

// 计算到目标点的方位角
double bearingToTarget = CoordinateTransform::calculateBearing(position, targetPosition);
```

## 注意事项

1. **性能考虑**：Eigen方法计算量较大，适合对精度要求高的应用
2. **内存使用**：Eigen对象会占用更多内存，但提供了更好的数值稳定性
3. **依赖管理**：需要正确配置Eigen库的依赖关系

## 扩展建议

1. **添加更多坐标系**：如ENU、NED等
2. **实现四元数旋转**：用于更复杂的姿态变换
3. **添加重力模型**：考虑地球重力场的变化
4. **实现卡尔曼滤波**：用于状态估计和滤波 