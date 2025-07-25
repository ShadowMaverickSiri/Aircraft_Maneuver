#include <iostream>
#include <memory>
#include <iomanip>
#include "AircraftModelLibrary.h"
#include "FighterJet.h"
#include "ManeuverModel.h"   // 新的机动模型接口
#include "AircraftModule.h"  // 新的功能模块接口
#include "CoordinateTransform.h"
#include "ImprovedCoordinateTransform.h"

// 示例：自定义干扰模块
class JammerModule : public AircraftModule {
public:
    std::string getModuleName() const override { return "Jammer"; }
    void activateJamming() { isActive = true; }
    void deactivateJamming() { isActive = false; }
    bool isJamming() const { return isActive; }
    void update(Aircraft& aircraft, double dt) override {
        if (isActive) {
            // 干扰逻辑示例
        }
    }
private:
    bool isActive = false;
};

int main() {
	std::string type = "fighter";
	std::string aircraftModel = "F-15";
	std::string maneuver;

	std::cout << "Choose aircraft type (fighter, passenger, uav): ";
	std::cin >> type;
	std::cout << "Enter aircraft model (e.g. F-15, Su-27): ";
	std::cin >> aircraftModel;
	std::cout << "Choose maneuver (loop, roll, s, advanced_s, snake, constant): ";
	std::cin >> maneuver;

	std::unique_ptr<Aircraft> aircraft;
	if (type == "fighter") {
		aircraft = std::make_unique<FighterJet>(aircraftModel);
	} else {
		std::cerr << "暂不支持该类型，默认创建FighterJet。" << std::endl;
		aircraft = std::make_unique<FighterJet>(aircraftModel);
	}

	// 添加干扰模块示例
	auto jammer = std::make_shared<JammerModule>();
	aircraft->addModule(jammer);
	jammer->activateJamming();

	// 设置飞机的初始位置
	aircraft->position = {116.4074, 39.9042, 1000.0};  // 北京上空1000米
	// 可选的其他初始位置：
	// aircraft->position = {121.4737, 31.2304, 1000.0};  // 上海上空1000米
	// aircraft->position = {-74.0060, 40.7128, 1000.0};  // 纽约上空1000米
	// aircraft->position = {-0.1278, 51.5074, 1000.0};   // 伦敦上空1000米

	// 设置参考位置（用于计算相对位置）
	aircraft->setReferencePosition(aircraft->position);

	try {
		aircraft->setManeuver(maneuver);
	}
	catch (const std::exception & e) {
		std::cerr << e.what() << "\n";
		return 1;
	}

	aircraft->velocity = { 200.0, 0.0, 0.0 }; // 初始速度：北向 200 m/s

	const double dt = 0.1;
	const int steps = 50;

	// 1. 创建机动模型
	auto maneuverModel = ManeuverModelFactory::createManeuverModel(maneuver);

	// 2. 设置到飞机
	aircraft->setManeuverModel(maneuverModel);

	// 3. 初始化参数
	ManeuverParameters params;
	params.turnRate = 0.3;
	params.period = 4.0;
	params.amplitude = 1.0;
	aircraft->initializeManeuver(params);

	std::cout << "\n=== 飞机机动仿真开始 ===" << std::endl;
	std::cout << "初始位置: 北京 (" << aircraft->position.latitude << "°, " 
	          << aircraft->position.longitude << "°, " << aircraft->position.altitude << "m)" << std::endl;
	std::cout << "机动类型: " << maneuver << std::endl;
	std::cout << "时间步长: " << dt << "s" << std::endl;
	std::cout << "仿真步数: " << steps << std::endl;
	std::cout << "=====================================" << std::endl;

	// 4. 仿真循环
	for (int i = 0; i < steps; ++i) {
		aircraft->updateModules(dt);      // 更新所有功能模块
		aircraft->updateManeuver(dt);      // 步进函数
		aircraft->updateKinematics(dt);    // 运动学更新
		
		// 计算坐标转换
		Vector3 ecefPos = aircraft->getECEFPosition();
		Vector3 localNUE = aircraft->getLocalNUEPosition();
		double distanceFromRef = aircraft->getDistanceFromReference();
		double bearingFromRef = aircraft->getBearingFromReference();
		
		// 输出位置信息
		std::cout << std::fixed << std::setprecision(6);
		std::cout << "Step " << i << " (t=" << (i+1)*dt << "s):\n";
		std::cout << "  地理坐标: Lat=" << aircraft->position.latitude 
		          << "°, Lon=" << aircraft->position.longitude 
		          << "°, Alt=" << aircraft->position.altitude << "m\n";
		
		std::cout << "  地球坐标系 (ECEF): X=" << ecefPos.north 
		          << "m, Y=" << ecefPos.up << "m, Z=" << ecefPos.east << "m\n";
		std::cout << "  当地地理系 (NUE): North=" << localNUE.north 
		          << "m, Up=" << localNUE.up << "m, East=" << localNUE.east << "m\n";
		std::cout << "  相对参考点: 距离=" << distanceFromRef/1000.0 
		          << "km, 方位角=" << bearingFromRef << "°\n";
		
		std::cout << "  速度: North=" << aircraft->velocity.north 
		          << "m/s, Up=" << aircraft->velocity.up 
		          << "m/s, East=" << aircraft->velocity.east << "m/s\n";
		std::cout << "  姿态: Pitch=" << aircraft->attitude.getPitchDegrees() 
		          << "°, Roll=" << aircraft->attitude.getRollDegrees() 
		          << "°, Yaw=" << aircraft->attitude.getYawDegrees() << "°\n";
		std::cout << std::endl;
	}

	std::cout << "=== 仿真完成 ===" << std::endl;
	std::cout << "最终位置: (" << aircraft->position.latitude << "°, " 
	          << aircraft->position.longitude << "°, " << aircraft->position.altitude << "m)" << std::endl;
	std::cout << "总飞行距离: " << aircraft->getDistanceFromReference()/1000.0 << " km" << std::endl;

	return 0;
}
