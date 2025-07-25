#include <iostream>
#include <iomanip>
#include <cmath>
#include "AircraftModelLibrary.h"
#include "FighterJet.h"

int main() {
    std::cout << "=== 坐标转换功能测试 ===" << std::endl;
    
    // 创建战斗机实例
    std::unique_ptr<Aircraft> aircraft = std::make_unique<FighterJet>("F-15");
    
    // 测试1：北京位置
    std::cout << "\n--- 测试1：北京位置 ---" << std::endl;
    aircraft->position = {39.9042, 116.4074, 1000.0};
    aircraft->setReferencePosition(aircraft->position);
    
    Vector3 ecef = aircraft->getECEFPosition();
    Vector3 localNUE = aircraft->getLocalNUEPosition();
    double distance = aircraft->getDistanceFromReference();
    double bearing = aircraft->getBearingFromReference();
    
    std::cout << "位置: (" << aircraft->position.latitude << "°, " 
              << aircraft->position.longitude << "°, " << aircraft->position.altitude << "m)" << std::endl;
    std::cout << "ECEF: (" << ecef.north << ", " << ecef.up << ", " << ecef.east << ") m" << std::endl;
    std::cout << "NUE: (" << localNUE.north << ", " << localNUE.up << ", " << localNUE.east << ") m" << std::endl;
    std::cout << "距离: " << distance << " m" << std::endl;
    std::cout << "方位角: " << bearing << "°" << std::endl;
    
    // 验证初始位置应该为0
    if (std::abs(distance) < 1e-6 && std::abs(localNUE.north) < 1e-6 && 
        std::abs(localNUE.up) < 1e-6 && std::abs(localNUE.east) < 1e-6) {
        std::cout << "✓ 初始位置测试通过" << std::endl;
    } else {
        std::cout << "✗ 初始位置测试失败" << std::endl;
        return 1;
    }
    
    // 测试2：上海位置
    std::cout << "\n--- 测试2：上海位置 ---" << std::endl;
    aircraft->position = {31.2304, 121.4737, 500.0};
    
    ecef = aircraft->getECEFPosition();
    localNUE = aircraft->getLocalNUEPosition();
    distance = aircraft->getDistanceFromReference();
    bearing = aircraft->getBearingFromReference();
    
    std::cout << "位置: (" << aircraft->position.latitude << "°, " 
              << aircraft->position.longitude << "°, " << aircraft->position.altitude << "m)" << std::endl;
    std::cout << "ECEF: (" << ecef.north << ", " << ecef.up << ", " << ecef.east << ") m" << std::endl;
    std::cout << "NUE: (" << localNUE.north << ", " << localNUE.up << ", " << localNUE.east << ") m" << std::endl;
    std::cout << "距离: " << distance/1000.0 << " km" << std::endl;
    std::cout << "方位角: " << bearing << "°" << std::endl;
    
    // 验证距离应该在合理范围内（北京到上海约1000km）
    if (distance > 900000 && distance < 1100000) {
        std::cout << "✓ 距离计算测试通过" << std::endl;
    } else {
        std::cout << "✗ 距离计算测试失败" << std::endl;
        return 1;
    }
    
    // 测试3：纽约位置
    std::cout << "\n--- 测试3：纽约位置 ---" << std::endl;
    aircraft->position = {40.7128, -74.0060, 2000.0};
    
    ecef = aircraft->getECEFPosition();
    localNUE = aircraft->getLocalNUEPosition();
    distance = aircraft->getDistanceFromReference();
    bearing = aircraft->getBearingFromReference();
    
    std::cout << "位置: (" << aircraft->position.latitude << "°, " 
              << aircraft->position.longitude << "°, " << aircraft->position.altitude << "m)" << std::endl;
    std::cout << "ECEF: (" << ecef.north << ", " << ecef.up << ", " << ecef.east << ") m" << std::endl;
    std::cout << "NUE: (" << localNUE.north << ", " << localNUE.up << ", " << localNUE.east << ") m" << std::endl;
    std::cout << "距离: " << distance/1000.0 << " km" << std::endl;
    std::cout << "方位角: " << bearing << "°" << std::endl;
    
    // 验证方位角应该在合理范围内
    if (bearing >= -180 && bearing <= 180) {
        std::cout << "✓ 方位角计算测试通过" << std::endl;
    } else {
        std::cout << "✗ 方位角计算测试失败" << std::endl;
        return 1;
    }
    
    // 测试4：ECEF坐标验证
    std::cout << "\n--- 测试4：ECEF坐标验证 ---" << std::endl;
    
    // 测试赤道位置
    aircraft->position = {0.0, 0.0, 1000.0};
    ecef = aircraft->getECEFPosition();
    
    std::cout << "赤道位置 (0°, 0°, 1000m):" << std::endl;
    std::cout << "ECEF: (" << ecef.north << ", " << ecef.up << ", " << ecef.east << ") m" << std::endl;
    
    // 验证赤道位置的ECEF坐标
    const double EARTH_RADIUS = 6378137.0;
    double expectedX = (EARTH_RADIUS + 1000.0);
    double expectedY = 0.0;
    double expectedZ = 0.0;
    
    if (std::abs(ecef.north - expectedX) < 1000 && 
        std::abs(ecef.up - expectedY) < 1000 && 
        std::abs(ecef.east - expectedZ) < 1000) {
        std::cout << "✓ ECEF坐标计算测试通过" << std::endl;
    } else {
        std::cout << "✗ ECEF坐标计算测试失败" << std::endl;
        return 1;
    }
    
    std::cout << "\n=== 所有测试通过！===" << std::endl;
    return 0;
} 