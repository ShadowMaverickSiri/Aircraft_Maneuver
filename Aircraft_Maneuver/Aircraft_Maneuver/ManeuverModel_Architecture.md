# 飞机机动模型架构设计

## 概述

本文档描述了将飞机机动模型重构为**初始化函数（initialize）**和**步进函数（update）**形式的架构设计。这种设计模式更适合模块化、实时仿真和状态管理。

## 架构设计

### 1. 核心类结构

```
ManeuverModel (抽象基类)
├── initialize(params)    // 初始化函数
├── update(aircraft, dt)  // 步进函数
├── getName()            // 获取机动名称
└── reset()              // 重置状态

具体机动模型类：
├── SManeuverModel
├── AdvancedSManeuverModel
├── SnakeManeuverModel
├── LoopManeuverModel
└── RollManeuverModel

Aircraft类增强：
├── setManeuverModel()    // 设置机动模型
├── initializeManeuver()  // 初始化机动
├── updateManeuver()      // 更新机动
└── resetManeuver()       // 重置机动
```

### 2. 数据结构

#### 机动参数结构体
```cpp
struct ManeuverParameters {
    double turnRate;        // 转向速率 (rad/s)
    double period;          // 周期 (秒)
    double amplitude;       // 幅度
    double climbRate;       // 爬升速率 (m/s)
    double altitudePeriod;  // 高度变化周期 (秒)
};
```

#### 机动状态结构体
```cpp
struct ManeuverState {
    double totalTime;       // 累积时间
    double currentPhase;    // 当前相位
    bool isInitialized;     // 是否已初始化
};
```

### 3. 设计模式

#### 策略模式
- 不同的机动模型实现相同的接口
- 可以在运行时切换机动策略

#### 工厂模式
- `ManeuverModelFactory` 负责创建具体的机动模型
- 提供默认参数配置

#### 状态模式
- 每个机动模型维护自己的状态
- 支持初始化和重置操作

## 使用方法

### 1. 基本使用流程

```cpp
// 1. 创建飞机
auto aircraft = std::make_unique<FighterJet>("F-15");

// 2. 创建机动模型
auto maneuverModel = ManeuverModelFactory::createManeuverModel("s");

// 3. 获取参数
auto params = ManeuverModelFactory::getDefaultParameters("s");

// 4. 设置机动模型
aircraft->setManeuverModel(maneuverModel);

// 5. 初始化机动
aircraft->initializeManeuver(params);

// 6. 仿真循环
for (int i = 0; i < steps; ++i) {
    aircraft->updateManeuver(dt);      // 步进函数
    aircraft->updateKinematics(dt);    // 更新运动学
}
```

### 2. 自定义参数

```cpp
ManeuverParameters customParams;
customParams.turnRate = 0.5;      // 更快的转向
customParams.period = 3.0;        // 更短的周期
customParams.amplitude = 1.2;     // 更大的幅度
customParams.climbRate = 10.0;    // 更快的爬升
customParams.altitudePeriod = 2.0; // 更短的高度周期

aircraft->initializeManeuver(customParams);
```

### 3. 状态查询

```cpp
// 获取机动状态
const auto& state = aircraft->getManeuverState();
std::cout << "总时间: " << state.totalTime << " s" << std::endl;
std::cout << "已初始化: " << state.isInitialized << std::endl;

// 获取机动参数
const auto& params = aircraft->getManeuverParameters();
std::cout << "转向速率: " << params.turnRate << " rad/s" << std::endl;
```

## 优势对比

### 传统方法 vs 新架构

| 特性 | 传统方法 | 新架构 |
|------|----------|--------|
| **状态管理** | 静态变量 | 显式状态对象 |
| **参数配置** | 硬编码 | 可配置参数 |
| **初始化** | 隐式 | 显式初始化 |
| **重置** | 不支持 | 支持重置 |
| **扩展性** | 有限 | 高度可扩展 |
| **测试性** | 困难 | 易于测试 |
| **实时性** | 一般 | 优秀 |

### 具体优势

1. **模块化设计**
   - 每个机动模型独立实现
   - 易于添加新的机动类型
   - 代码复用性高

2. **状态管理**
   - 显式的状态跟踪
   - 支持暂停/恢复
   - 便于调试和监控

3. **参数化配置**
   - 运行时参数调整
   - 支持参数预设
   - 便于优化和调优

4. **实时性能**
   - 步进函数设计适合实时仿真
   - 最小化计算开销
   - 支持固定时间步长

## 实现细节

### 1. 初始化函数

```cpp
void SManeuverModel::initialize(const ManeuverParameters& params) {
    this->params = params;    // 保存参数
    totalTime = 0.0;          // 重置时间
    // 其他初始化操作...
}
```

### 2. 步进函数

```cpp
void SManeuverModel::update(Aircraft& aircraft, double dt) {
    totalTime += dt;          // 更新时间
    
    // 计算当前阶段的转向角度
    double phase = 2.0 * M_PI * totalTime / params.period;
    double turnAngle = params.turnRate * sin(phase) * dt;
    
    // 应用转向到飞机速度
    applyTurn(aircraft, turnAngle);
}
```

### 3. 状态重置

```cpp
void SManeuverModel::reset() {
    totalTime = 0.0;          // 重置时间
    // 重置其他状态变量...
}
```

## 扩展指南

### 1. 添加新的机动模型

```cpp
class NewManeuverModel : public ManeuverModel {
public:
    void initialize(const ManeuverParameters& params) override {
        // 初始化实现
    }
    
    void update(Aircraft& aircraft, double dt) override {
        // 步进函数实现
    }
    
    std::string getName() const override {
        return "New Maneuver";
    }
    
    void reset() override {
        // 重置实现
    }
};
```

### 2. 在工厂中注册

```cpp
std::shared_ptr<ManeuverModel> ManeuverModelFactory::createManeuverModel(const std::string& name) {
    if (name == "new_maneuver") {
        return std::make_shared<NewManeuverModel>();
    }
    // 其他机动类型...
}
```

### 3. 添加默认参数

```cpp
ManeuverParameters ManeuverModelFactory::getDefaultParameters(const std::string& maneuverType) {
    if (maneuverType == "new_maneuver") {
        ManeuverParameters params;
        params.turnRate = 0.3;
        params.period = 4.0;
        // 设置其他参数...
        return params;
    }
    // 其他机动类型...
}
```

## 性能考虑

### 1. 内存管理
- 使用智能指针管理机动模型生命周期
- 避免频繁的内存分配

### 2. 计算优化
- 预计算常用值（如sin/cos）
- 使用查找表优化三角函数计算

### 3. 实时性保证
- 步进函数时间复杂度O(1)
- 避免动态内存分配
- 使用固定时间步长

## 总结

新的机动模型架构提供了：

1. **清晰的接口设计**：初始化函数和步进函数分离
2. **灵活的参数配置**：运行时可调整的机动参数
3. **完善的状态管理**：显式的状态跟踪和重置
4. **良好的扩展性**：易于添加新的机动类型
5. **优秀的实时性**：适合实时仿真应用

这种设计模式使得飞机机动仿真更加模块化、可配置和可维护，同时保持了良好的性能特性。 