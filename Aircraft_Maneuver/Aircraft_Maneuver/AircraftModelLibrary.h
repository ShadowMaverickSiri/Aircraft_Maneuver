#ifndef AIRCRAFT_MODEL_LIBRARY_H
#define AIRCRAFT_MODEL_LIBRARY_H

#include <string>
#include <functional>
#include <map>
#include <memory>
#include <cmath>
#include <vector>
#include "AircraftModule.h"

// 定义圆周率
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 结构体：用经纬高表示位置
struct GeoPosition {
	double longitude;  // 经度 (度)
	double latitude;   // 纬度 (度)
	double altitude;   // 高度 (米，海平面以上)
};

// 结构体：用北-上-东坐标系表示速度
struct Vector3 {
	double north; // 北向速度 (m/s)
	double up;    // 垂直速度 (m/s)
	double east;  // 东向速度 (m/s)
};

// 结构体：包含姿态角信息
struct AttitudeAngles {
	double pitch;    // 俯仰角 (弧度) - 头向下偏转角度
	double roll;     // 滚转角 (弧度) - 机身向左偏转角度
	double yaw;      // 偏航角 (弧度) - 头向右偏转角度
	
	AttitudeAngles() : pitch(0.0), roll(0.0), yaw(0.0) {}
	
	// 转换为度数
	double getPitchDegrees() const { return pitch * 180.0 / M_PI; }
	double getRollDegrees() const { return roll * 180.0 / M_PI; }
	double getYawDegrees() const { return yaw * 180.0 / M_PI; }
	
	// 从度数设置
	void setPitchDegrees(double degrees) { pitch = degrees * M_PI / 180.0; }
	void setRollDegrees(double degrees) { roll = degrees * M_PI / 180.0; }
	void setYawDegrees(double degrees) { yaw = degrees * M_PI / 180.0; }
};

// 结构体：包含飞机性能参数
struct AircraftPerformance {
	double maxTurnRate;      // 最大转弯率 (弧度/秒)	
	double maxClimbRate;     // 最大爬升率 (米/秒)
	double maxRollRate;      // 最大滚转率 (弧度/秒)
	double maxPitchRate;     // 最大俯仰率 (弧度/秒)		
	double maxThrust;        // 最大推力 (牛顿)
	double dragCoefficient;  // 阻力系数
	double wingArea;         // 机翼面积 (平方米)
	double mass;             // 飞机质量 (千克)
	
	AircraftPerformance() : maxTurnRate(0.5), maxClimbRate(50.0), 
	                       maxRollRate(2.0), maxPitchRate(1.0),
	                       maxThrust(200000.0), dragCoefficient(0.02),
	                       wingArea(50.0), mass(10000.0) {}
};

// 只保留GeoPosition, Vector3, AttitudeAngles, AircraftPerformance, Aircraft等基础结构体和类
// 移除ManeuverModel、ManeuverParameters、ManeuverState等机动相关内容
class Aircraft {
public:
	using ManeuverFunc = std::function<void(Aircraft&, double)>;

	Aircraft(const std::string& type, const std::string& model);
	virtual ~Aircraft();

	// 获取飞机类型
	std::string getType() const { return type; }
	// 获取飞机型号
	std::string getModel() const { return model; }

	GeoPosition position;    // 位置
	Vector3 velocity;        // 速度
	AttitudeAngles attitude; // 姿态
	AircraftPerformance performance; // 性能

	// 根据当前状态计算加速度，然后根据加速度更新位置
	virtual void updateKinematics(double dt);

	// 计算当前加速度 (m/s^2)
	virtual Vector3 computeAcceleration() const = 0;
	
	// 更新姿态
	virtual void updateAttitude(double dt);

	// 传统机动方法（保持向后兼容）
	void setManeuver(const std::string& name);         // 设置机动
	void performManeuver(double dt);                   // 执行当前机动
	static void registerManeuver(const std::string& name, ManeuverFunc func); // 注册机动

	// 新的机动模型方法
	void setManeuverModel(std::shared_ptr<ManeuverModel> model);  // 设置机动模型
	void initializeManeuver(const ManeuverParameters& params);    // 初始化机动
	void updateManeuver(double dt);                               // 更新机动
	void resetManeuver();                                         // 重置机动状态
	
	// 获取当前机动状态
	const ManeuverState& getManeuverState() const { return maneuverState; }
	const ManeuverParameters& getManeuverParameters() const { return maneuverParams; }
	
	// 获取性能参数
	const AircraftPerformance& getPerformance() const { return performance; }

	// 坐标转换相关方法
	void setReferencePosition(const GeoPosition& refPos);  // 设置参考位置
	GeoPosition getReferencePosition() const { return referencePosition; }
	
	// 获取地球坐标系位置（简化版本，不依赖Eigen）
	Vector3 getECEFPosition() const;
	
	// 获取相对于参考点的当地地理系位置
	Vector3 getLocalNUEPosition() const;
	
	// 获取相对于参考点的距离和方位角
	double getDistanceFromReference() const;
	double getBearingFromReference() const;

	// 功能模块管理接口
	void addModule(std::shared_ptr<AircraftModule> module) {
		modules.push_back(module);
	}
	template<typename T>
	std::shared_ptr<T> getModule() const {
		for (const auto& m : modules) {
			auto casted = std::dynamic_pointer_cast<T>(m);
			if (casted) return casted;
		}
		return nullptr;
	}
	void updateModules(double dt) {
		for (auto& m : modules) {
			m->update(*this, dt);
		}
	}

protected:
	std::string type;   // 飞机类型（如fighter, passenger, uav等）
	std::string model;  // 飞机型号（如F-15, Su-27等）
	ManeuverFunc currentManeuver;
	static std::map<std::string, ManeuverFunc> maneuvers; // 机动函数映射
	
	// 新的机动模型成员
	std::shared_ptr<ManeuverModel> currentManeuverModel;
	ManeuverState maneuverState;
	ManeuverParameters maneuverParams;
	
	// 坐标转换相关成员
	GeoPosition referencePosition;  // 参考位置（用于计算相对位置）
	std::vector<std::shared_ptr<AircraftModule>> modules;
};

// 根据速度和时间步长更新位置
GeoPosition updateGeoPosition(const GeoPosition& pos, const Vector3& velocity, double dt);

#endif // AIRCRAFT_MODEL_LIBRARY_H
