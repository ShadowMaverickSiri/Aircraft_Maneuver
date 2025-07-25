#include <iostream>
#include "AircraftModelLibrary.h"
#include "FighterJet.h"
#include "ManeuverModel.h"

int main() {
    std::cout << "=== 编译测试程序 ===" << std::endl;
    
    try {
        // 测试创建飞机
        std::unique_ptr<Aircraft> aircraft = std::make_unique<FighterJet>("F-15");
        std::cout << "✓ 飞机创建成功" << std::endl;
        
        // 测试机动模型工厂
        auto loopModel = ManeuverModelFactory::createManeuverModel("loop");
        auto rollModel = ManeuverModelFactory::createManeuverModel("roll");
        auto sModel = ManeuverModelFactory::createManeuverModel("s");
        std::cout << "✓ 机动模型创建成功" << std::endl;
        
        // 测试设置机动模型
        aircraft->setManeuverModel(loopModel);
        std::cout << "✓ 机动模型设置成功" << std::endl;
        
        // 测试初始化
        ManeuverParameters params;
        aircraft->initializeManeuver(params);
        std::cout << "✓ 机动初始化成功" << std::endl;
        
        // 测试更新
        aircraft->updateManeuver(0.1);
        aircraft->updateKinematics(0.1);
        std::cout << "✓ 更新函数调用成功" << std::endl;
        
        std::cout << "\n=== 所有测试通过 ===" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "✗ 测试失败: " << e.what() << std::endl;
        return 1;
    }
} 