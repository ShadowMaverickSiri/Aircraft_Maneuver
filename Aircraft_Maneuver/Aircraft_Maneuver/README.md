# 飞机机动仿真程序

## 概述

这是一个现代C++飞机机动仿真平台，支持多种机动样式、模块化功能扩展、精确坐标转换和完善的单元测试体系。项目采用C++17标准，结构清晰，易于扩展和维护。

---

## 目录结构

```
Aircraft_Maneuver/
  Aircraft_Maneuver/
    main.cpp                        # 主程序入口
    AircraftModelLibrary.h/.cpp     # 飞机基础结构体、基类、通用接口
    FighterJet.h/.cpp               # 战斗机实现
    ManeuverModel.h/.cpp            # 机动模型接口与所有机动模型实现
    AircraftModule.h                # 功能模块基类接口
    CoordinateTransform.h/.cpp      # 坐标转换相关
    ImprovedCoordinateTransform.h/.cpp # 改进坐标转换
    EulerAngleCalculation.h/.cpp    # 欧拉角计算
    tests/
      test_aircraft_basic.cpp           # 飞机基础功能单元测试
      test_coordinate_transform.cpp     # 坐标转换单元测试
      test_compile.cpp                  # 编译/接口完整性测试
    examples/
      example_maneuver_usage.cpp        # 机动模型用法演示
    CMakeLists.txt                 # CMake工程配置
    ...（其它文档、说明等）
```

---

## 核心模块说明

### 1. AircraftModelLibrary.h/.cpp
- 定义飞机基础结构体（GeoPosition, Vector3, AttitudeAngles, AircraftPerformance）
- `Aircraft`基类，支持机动模型、功能模块挂载、运动学更新等

### 2. FighterJet.h/.cpp
- 战斗机具体实现，重写加速度计算

### 3. ManeuverModel.h/.cpp
- 机动模型接口（策略模式）与所有机动模型实现（S型、筋斗、横滚、破S、英麦曼、桶滚、置尾降高逃逸、L机动、定速定高等）
- `ManeuverModelFactory`工厂，支持按名称创建模型和获取默认参数

### 4. AircraftModule.h
- 功能模块基类接口，支持干扰、武器、传感器等模块扩展

### 5. CoordinateTransform.h/.cpp, ImprovedCoordinateTransform.h/.cpp
- 精确的地理坐标、ECEF、NUE等坐标转换与距离/方位角计算

### 6. EulerAngleCalculation.h/.cpp
- 欧拉角（俯仰、滚转、偏航）计算工具

---

## 测试与示例体系

### tests/
- `test_aircraft_basic.cpp`：飞机创建、属性赋值、运动学、坐标转换等基础功能测试
- `test_coordinate_transform.cpp`：多地理点的ECEF、NUE、距离、方位角等坐标转换单元测试
- `test_compile.cpp`：机动模型工厂、设置、初始化、步进等接口完整性测试

### examples/
- `example_maneuver_usage.cpp`：交互式选择机动类型，展示参数、仿真循环、输出轨迹

---

## 编译与运行

### 环境要求
- C++17 兼容编译器（如VS2019、g++ 7.0+）
- CMake 3.10 或更高
- （可选）Eigen库用于高精度坐标转换

### 编译步骤

1. **创建构建目录**
   ```sh
   cd Aircraft_Maneuver/Aircraft_Maneuver
   mkdir build && cd build
   cmake ..
   cmake --build . --config Release
   ```

2. **可执行文件说明**
   - `Aircraft_Maneuver`：主程序
   - `tests/test_aircraft_basic`、`tests/test_coordinate_transform`、`tests/test_compile`：单元测试
   - `examples/example_maneuver_usage`：机动模型用法演示

3. **运行方法**
   ```sh
   ./Aircraft_Maneuver
   ./tests/test_aircraft_basic
   ./tests/test_coordinate_transform
   ./examples/example_maneuver_usage
   ```

4. **VS2019用户**
   - 用“打开本地文件夹”方式打开项目根目录，CMake会自动识别所有目标。

---

## 主要用法示例

### 1. 主程序（main.cpp）
```cpp
std::unique_ptr<Aircraft> aircraft = std::make_unique<FighterJet>("F-15");
auto model = ManeuverModelFactory::createManeuverModel("loop");
auto params = ManeuverModelFactory::getDefaultParameters("loop");
aircraft->setManeuverModel(model);
aircraft->initializeManeuver(params);
for (int i = 0; i < steps; ++i) {
    aircraft->updateManeuver(dt);
    aircraft->updateKinematics(dt);
}
```

### 2. 挂载功能模块
```cpp
class JammerModule : public AircraftModule { ... };
aircraft->addModule(std::make_shared<JammerModule>());
```

### 3. 单元测试（test_aircraft_basic.cpp）
```cpp
// 测试飞机创建、属性赋值、运动学、坐标转换等
```

### 4. 机动模型用法演示（example_maneuver_usage.cpp）
```cpp
// 交互式选择机动类型，展示参数、仿真循环、输出轨迹
```

---

## 扩展开发建议

### 1. 添加新机动模型
- 在`ManeuverModel.h`声明新模型类，在`ManeuverModel.cpp`实现其算法
- 在`ManeuverModelFactory`注册新类型

### 2. 添加新功能模块
- 继承`AircraftModule`，实现接口
- 在主程序或测试中`addModule`挂载

### 3. 添加新测试
- 在`tests/`目录下新建测试文件，按需编写断言和输出
- 用CMake添加新测试目标

---

## 常见问题与故障排除

- **找不到头文件/链接错误**：检查CMakeLists.txt文件路径和include路径
- **运行时异常**：检查机动类型名称、参数设置、输入格式
- **中文乱码**：确保源文件保存为UTF-8编码
- **Eigen相关报错**：如需高精度坐标转换，请正确安装并配置Eigen库

---

## 许可证

本项目采用MIT许可证。

---

## 联系与贡献

如有建议、bug反馈或希望贡献代码，请通过GitHub Issue或Pull Request联系维护者。 