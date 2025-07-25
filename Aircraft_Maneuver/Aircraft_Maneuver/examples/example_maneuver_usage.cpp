#include <iostream>
#include <memory>
#include <iomanip>
#include "AircraftModelLibrary.h"
#include "FighterJet.h"
#include "ManeuverModel.h"

void printPosition(const std::string& label, const GeoPosition& pos) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << label << ": Lat=" << pos.latitude << "°, Lon=" << pos.longitude 
              << "°, Alt=" << pos.altitude << "m" << std::endl;
}

void printVelocity(const std::string& label, const Vector3& vel) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << label << ": North=" << vel.north << " m/s, Up=" << vel.up 
              << " m/s, East=" << vel.east << " m/s" << std::endl;
}

int main() {
    std::cout << "=== 机动模型示例（初始化函数 + 步进函数） ===" << std::endl;
    
    // 创建战斗机
    std::unique_ptr<Aircraft> aircraft = std::make_unique<FighterJet>("F-15");
    aircraft->velocity = {200.0, 0.0, 0.0}; // 初始速度：向北200 m/s
    
    std::cout << "初始状态：" << std::endl;
    printPosition("位置", aircraft->position);
    printVelocity("速度", aircraft->velocity);
    
    // 选择机动类型
    std::string maneuverType;
    std::cout << "\n选择机动类型 (s, s_advanced, snake, loop, roll): ";
    std::cin >> maneuverType;
    
    try {
        // 使用工厂创建机动模型
        auto maneuverModel = ManeuverModelFactory::createManeuverModel(maneuverType);
        
        // 获取默认参数
        auto params = ManeuverModelFactory::getDefaultParameters(maneuverType);
        
        std::cout << "\n机动参数：" << std::endl;
        std::cout << "转向速率: " << params.turnRate << " rad/s" << std::endl;
        std::cout << "周期: " << params.period << " s" << std::endl;
        std::cout << "幅度: " << params.amplitude << std::endl;
        std::cout << "爬升速率: " << params.climbRate << " m/s" << std::endl;
        std::cout << "高度周期: " << params.altitudePeriod << " s" << std::endl;
        
        // 设置机动模型
        aircraft->setManeuverModel(maneuverModel);
        
        // 初始化机动
        aircraft->initializeManeuver(params);
        
        std::cout << "\n开始仿真..." << std::endl;
        std::cout << "时间(s) | 纬度(°) | 经度(°) | 高度(m) | 北向速度(m/s) | 垂直速度(m/s) | 东向速度(m/s)" << std::endl;
        std::cout << "--------|---------|---------|---------|---------------|---------------|---------------" << std::endl;
        
        const double dt = 0.1;  // 时间步长
        const int steps = 50;   // 仿真步数
        
        for (int i = 0; i < steps; ++i) {
            double currentTime = (i + 1) * dt;
            
            // 更新机动（步进函数）
            aircraft->updateManeuver(dt);
            
            // 更新运动学
            aircraft->updateKinematics(dt);
            
            // 输出状态
            const auto& pos = aircraft->position;
            const auto& vel = aircraft->velocity;
            
            std::cout << std::fixed << std::setprecision(3);
            std::cout << std::setw(7) << currentTime << " | "
                      << std::setw(7) << pos.latitude << " | "
                      << std::setw(7) << pos.longitude << " | "
                      << std::setw(7) << pos.altitude << " | "
                      << std::setw(13) << vel.north << " | "
                      << std::setw(13) << vel.up << " | "
                      << std::setw(13) << vel.east << std::endl;
        }
        
        // 显示最终状态
        std::cout << "\n最终状态：" << std::endl;
        printPosition("位置", aircraft->position);
        printVelocity("速度", aircraft->velocity);
        
        // 显示机动状态
        const auto& maneuverState = aircraft->getManeuverState();
        std::cout << "\n机动状态：" << std::endl;
        std::cout << "总时间: " << maneuverState.totalTime << " s" << std::endl;
        std::cout << "当前相位: " << maneuverState.currentPhase << " rad" << std::endl;
        std::cout << "已初始化: " << (maneuverState.isInitialized ? "是" : "否") << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 