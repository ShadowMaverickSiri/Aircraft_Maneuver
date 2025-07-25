# 飞机机动仿真程序使用指南

## 程序概览

本工程包含多个可执行程序，每个程序都有特定的用途。CMake会自动编译所有存在的程序，您可以根据需要运行不同的程序。

## 程序列表

### 1. 主程序 - Aircraft_Maneuver
**文件**: `main.cpp`
**用途**: 主要的飞机机动仿真程序
**功能**:
- 完整的飞机机动仿真
- 实时坐标转换显示
- 支持所有机动类型
- 显示ECEF和NUE坐标

**使用场景**:
- 进行完整的飞机机动仿真
- 查看飞机的实时位置和坐标信息
- 测试各种机动类型

**运行方式**:
```bash
./Aircraft_Maneuver
```

### 2. 机动模型示例 - ManeuverModelExample
**文件**: `ManeuverModelExample.cpp`
**用途**: 演示新的机动模型架构
**功能**:
- 展示初始化函数和步进函数的使用
- 详细的参数和状态信息显示
- 表格形式的状态输出

**使用场景**:
- 学习新的机动模型架构
- 查看详细的仿真参数
- 调试机动模型

**运行方式**:
```bash
./ManeuverModelExample
```

### 3. 编译测试程序 - test_compile
**文件**: `test_compile.cpp`
**用途**: 验证所有组件是否正常工作
**功能**:
- 测试所有机动模型的创建
- 验证基本功能
- 检查编译是否成功

**使用场景**:
- 验证编译是否成功
- 检查基本功能
- 快速测试

**运行方式**:
```bash
./test_compile
```

### 4. 坐标转换测试程序 - test_coordinate_transform
**文件**: `test_coordinate_transform.cpp`
**用途**: 专门测试坐标转换功能
**功能**:
- 验证ECEF坐标计算
- 测试NUE坐标转换
- 验证距离和方位角计算
- 边界条件测试

**使用场景**:
- 验证坐标转换功能
- 调试坐标计算问题
- 确保计算精度

**运行方式**:
```bash
./test_coordinate_transform
```

### 5. 坐标转换演示程序 - CoordinateTransformDemo
**文件**: `CoordinateTransformDemo.cpp`
**用途**: 演示坐标转换功能
**功能**:
- 展示各种坐标转换
- 测试不同位置的坐标计算
- 验证距离和方位角计算
- 实际飞行过程模拟

**使用场景**:
- 学习坐标转换功能
- 查看不同位置的坐标
- 验证计算精度

**运行方式**:
```bash
./CoordinateTransformDemo
```

### 6. 坐标转换示例程序 - CoordinateTransformExample (可选)
**文件**: `CoordinateTransformExample.cpp`
**用途**: 使用Eigen库的精确坐标转换
**功能**:
- 使用Eigen库进行高精度计算
- 完整的ECEF坐标转换
- 需要Eigen库支持

**使用场景**:
- 需要最高精度坐标转换
- 已安装Eigen库
- 研究精确坐标转换

**运行方式**:
```bash
./CoordinateTransformExample
```

## 程序选择指南

### 🚀 快速开始
如果您想快速体验飞机机动仿真：
```bash
./Aircraft_Maneuver
```

### 🔧 验证安装
如果您想验证程序是否正确安装：
```bash
./test_compile
./test_coordinate_transform
```

### 📚 学习功能
如果您想学习各种功能：
```bash
# 学习机动模型
./ManeuverModelExample

# 学习坐标转换
./CoordinateTransformDemo
```

### 🎯 特定用途
- **研究机动模型**: `ManeuverModelExample`
- **测试坐标转换**: `test_coordinate_transform`
- **演示坐标功能**: `CoordinateTransformDemo`
- **高精度坐标**: `CoordinateTransformExample` (需要Eigen)

## CMake管理机制

### 自动编译
CMake会自动检测存在的源文件并编译相应的程序：

```cmake
# 主程序（总是编译）
add_executable(Aircraft_Maneuver ${BASE_SOURCES})

# 机动模型示例（总是编译）
add_executable(ManeuverModelExample ...)

# 测试程序（总是编译）
add_executable(test_compile ...)
add_executable(test_coordinate_transform ...)

# 坐标转换演示（如果文件存在）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/CoordinateTransformDemo.cpp")
    add_executable(CoordinateTransformDemo ...)
endif()

# 坐标转换示例（如果文件存在且需要Eigen）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/CoordinateTransformExample.cpp")
    add_executable(CoordinateTransformExample ...)
endif()
```

### 条件编译
- **基础程序**: 总是编译（主程序、测试程序）
- **演示程序**: 如果源文件存在则编译
- **Eigen程序**: 如果文件存在且Eigen库可用则编译

### 编译输出
编译时会显示哪些程序将被构建：
```
-- Coordinate transform demo will be built
-- Coordinate transform example will be built
-- Eigen library found - coordinate transform features will be available
```

## 程序依赖关系

### 核心库文件
所有程序都依赖以下核心文件：
- `AircraftModelLibrary.cpp/h` - 飞机基类
- `FighterJet.cpp/h` - 战斗机实现
- `EulerAngleCalculation.cpp/h` - 欧拉角计算

### 可选依赖
- `ManeuverModels.cpp/h` - 机动模型（部分程序需要）
- `ImprovedCoordinateTransform.cpp/h` - 改进坐标转换
- `CoordinateTransform.cpp/h` - Eigen坐标转换（需要Eigen库）

## 使用建议

### 1. 首次使用
```bash
# 1. 编译项目
mkdir build && cd build
cmake .. && make

# 2. 验证安装
./test_compile
./test_coordinate_transform

# 3. 运行主程序
./Aircraft_Maneuver
```

### 2. 开发调试
```bash
# 测试基本功能
./test_compile

# 测试坐标转换
./test_coordinate_transform

# 查看详细输出
./ManeuverModelExample
```

### 3. 功能学习
```bash
# 学习机动模型
./ManeuverModelExample

# 学习坐标转换
./CoordinateTransformDemo
```

### 4. 性能测试
```bash
# 完整仿真
./Aircraft_Maneuver

# 高精度坐标（如果可用）
./CoordinateTransformExample
```

## 故障排除

### 程序不存在
如果某个程序没有编译：
1. 检查源文件是否存在
2. 查看CMake输出信息
3. 检查依赖库是否安装

### 运行错误
如果程序运行出错：
1. 先运行测试程序验证基本功能
2. 检查输入参数是否正确
3. 查看错误信息

### 精度问题
如果坐标精度不够：
1. 使用`CoordinateTransformExample`（需要Eigen）
2. 检查计算参数
3. 验证输入数据

## 总结

这个工程采用模块化设计，每个程序都有特定的用途：

- **主程序**: 完整仿真体验
- **测试程序**: 验证功能正确性
- **演示程序**: 学习特定功能
- **示例程序**: 展示高级特性

CMake会自动管理这些程序，您可以根据需要选择运行相应的程序。这种设计使得程序既易于使用，又便于开发和测试。 