#include <iostream>
#include "AircraftModelLibrary.h"
#include "FighterJet.h"

int main() {
    std::cout << "=== 简单功能测试 ===" << std::endl;
    
    try {
        // 创建战斗机
        std::unique_ptr<Aircraft> aircraft = std::make_unique<FighterJet>("F-15");
        
        // 设置位置
        aircraft->position = {116.4074, 39.9042, 1000.0};
        aircraft->setReferencePosition(aircraft->position);
        
        std::cout << "飞机创建成功" << std::endl;
        std::cout << "位置: (" << aircraft->position.latitude << ", " 
                  << aircraft->position.longitude << ", " << aircraft->position.altitude << ")" << std::endl;
        
        // 测试坐标转换
        Vector3 ecef = aircraft->getECEFPosition();
        Vector3 localNUE = aircraft->getLocalNUEPosition();
        double distance = aircraft->getDistanceFromReference();
        double bearing = aircraft->getBearingFromReference();
        
        std::cout << "ECEF坐标: (" << ecef.north << ", " << ecef.up << ", " << ecef.east << ")" << std::endl;
        std::cout << "NUE坐标: (" << localNUE.north << ", " << localNUE.up << ", " << localNUE.east << ")" << std::endl;
        std::cout << "距离: " << distance << " m" << std::endl;
        std::cout << "方位角: " << bearing << "°" << std::endl;
        
        // 测试运动学更新
        aircraft->velocity = {200.0, 10.0, 0.0};
        aircraft->updateKinematics(0.1);
        
        std::cout << "更新后位置: (" << aircraft->position.latitude << ", " 
                  << aircraft->position.longitude << ", " << aircraft->position.altitude << ")" << std::endl;
        
        std::cout << "=== 测试通过 ===" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
} 