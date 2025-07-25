#include "AircraftModelLibrary.h"
#include "EulerAngleCalculation.h"
#include <cmath>
#include <stdexcept>
#include <sstream>

// 地球半径 (单位：米)
const double EARTH_RADIUS = 6371000.0;

// 静态变量初始化
std::map<std::string, Aircraft::ManeuverFunc> Aircraft::maneuvers;

Aircraft::Aircraft(const std::string& type, const std::string& model)
    : type(type), model(model),
      position{ 0.0, 0.0, 0.0 },
      velocity{ 0.0, 0.0, 0.0 },
      attitude(),
      performance(),
      currentManeuver(nullptr),
      currentManeuverModel(nullptr),
      maneuverState(),
      maneuverParams() {}

Aircraft::~Aircraft() {}

void Aircraft::updateKinematics(double dt) {
	Vector3 a = computeAcceleration();

	// 速度更新
	velocity.north += a.north * dt;
	velocity.up += a.up * dt;
	velocity.east += a.east * dt;

	// 根据当前速度更新位置
	position = updateGeoPosition(position, velocity, dt);
}

// 传统机动方法（保持向后兼容）
void Aircraft::registerManeuver(const std::string& name, ManeuverFunc func) {
	maneuvers[name] = func;
}

void Aircraft::setManeuver(const std::string& name) {
	auto it = maneuvers.find(name);
	if (it == maneuvers.end()) {
		std::ostringstream oss;
		oss << "Unknown maneuver: " << name << "\nAvailable maneuvers: ";
		for (const auto& pair : maneuvers)
			oss << pair.first << " ";
		throw std::invalid_argument(oss.str());
	}
	currentManeuver = it->second;
}

void Aircraft::performManeuver(double dt) {
	if (currentManeuver)
		currentManeuver(*this, dt);
}

// 新的机动模型方法
void Aircraft::setManeuverModel(std::shared_ptr<ManeuverModel> model) {
	currentManeuverModel = model;
	if (model) {
		model->initialize(maneuverParams);
		maneuverState.isInitialized = true;
	}
}

void Aircraft::initializeManeuver(const ManeuverParameters& params) {
	maneuverParams = params;
	maneuverState.reset();
	maneuverState.isInitialized = true;
	
	if (currentManeuverModel) {
		currentManeuverModel->initialize(params);
	}
}

void Aircraft::updateManeuver(double dt) {
	if (currentManeuverModel && maneuverState.isInitialized) {
		currentManeuverModel->update(*this, dt);
		maneuverState.totalTime += dt;
	}
}

void Aircraft::resetManeuver() {
	maneuverState.reset();
	if (currentManeuverModel) {
		currentManeuverModel->reset();
	}
}

void Aircraft::updateAttitude(double dt) {
	// 从速度向量计算基本姿态角
	attitude = EulerAngleCalculator::calculateFromVelocity(velocity);
}

// 坐标转换相关方法实现
void Aircraft::setReferencePosition(const GeoPosition& refPos) {
	referencePosition = refPos;
}

Vector3 Aircraft::getECEFPosition() const {
	// 简化的ECEF坐标计算（不依赖Eigen）
	const double EARTH_RADIUS = 6371000.0;
	const double PI = 3.14159265358979323846;
	
	double lat = position.latitude * PI / 180.0;  // 转换为弧度
	double lon = position.longitude * PI / 180.0;
	double h = position.altitude;
	
	// 简化的椭球体参数（WGS84）
	const double a = 6378137.0;  // 长半轴
	const double e2 = 0.006694379990141316;  // 第一偏心率平方
	
	double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));  // 卯酉圈曲率半径
	
	Vector3 ecef;
	ecef.north = (N + h) * cos(lat) * cos(lon);  // X分量
	ecef.up = (N + h) * cos(lat) * sin(lon);     // Y分量
	ecef.east = (N * (1 - e2) + h) * sin(lat);   // Z分量
	
	return ecef;
}

Vector3 Aircraft::getLocalNUEPosition() const {
	// 计算相对于参考点的NUE位置
	Vector3 refECEF = getECEFPosition();  // 当前飞机ECEF位置
	
	// 计算参考点的ECEF位置
	const double EARTH_RADIUS = 6371000.0;
	const double PI = 3.14159265358979323846;
	
	double refLat = referencePosition.latitude * PI / 180.0;
	double refLon = referencePosition.longitude * PI / 180.0;
	double refH = referencePosition.altitude;
	
	const double a = 6378137.0;
	const double e2 = 0.006694379990141316;
	
	double refN = a / sqrt(1 - e2 * sin(refLat) * sin(refLat));
	
	Vector3 refECEFPos;
	refECEFPos.north = (refN + refH) * cos(refLat) * cos(refLon);
	refECEFPos.up = (refN + refH) * cos(refLat) * sin(refLon);
	refECEFPos.east = (refN * (1 - e2) + refH) * sin(refLat);
	
	// 计算相对位置
	Vector3 relativeECEF;
	relativeECEF.north = refECEF.north - refECEFPos.north;
	relativeECEF.up = refECEF.up - refECEFPos.up;
	relativeECEF.east = refECEF.east - refECEFPos.east;
	
	// 转换为NUE坐标系
	Vector3 localNUE;
	
	// 构建旋转矩阵（从ECEF到NUE）
	double sinLat = sin(refLat);
	double cosLat = cos(refLat);
	double sinLon = sin(refLon);
	double cosLon = cos(refLon);
	
	// 旋转矩阵应用
	localNUE.north = -sinLat * cosLon * relativeECEF.north - sinLat * sinLon * relativeECEF.up + cosLat * relativeECEF.east;
	localNUE.up = cosLat * cosLon * relativeECEF.north + cosLat * sinLon * relativeECEF.up + sinLat * relativeECEF.east;
	localNUE.east = -sinLon * relativeECEF.north + cosLon * relativeECEF.up;
	
	return localNUE;
}

double Aircraft::getDistanceFromReference() const {
	// 计算到参考点的距离
	double dLat = position.latitude - referencePosition.latitude;
	double dLon = position.longitude - referencePosition.longitude;
	double dAlt = position.altitude - referencePosition.altitude;
	
	// 转换为弧度
	const double PI = 3.14159265358979323846;
	dLat *= PI / 180.0;
	dLon *= PI / 180.0;
	
	// 使用Haversine公式计算水平距离
	double lat1 = referencePosition.latitude * PI / 180.0;
	double lat2 = position.latitude * PI / 180.0;
	
	double a = sin(dLat/2) * sin(dLat/2) + cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	
	const double EARTH_RADIUS = 6371000.0;
	double horizontalDistance = EARTH_RADIUS * c;
	
	// 计算总距离（包括高度）
	return sqrt(horizontalDistance * horizontalDistance + dAlt * dAlt);
}

double Aircraft::getBearingFromReference() const {
	// 计算从参考点到当前位置的方位角
	double lat1 = referencePosition.latitude * 3.14159265358979323846 / 180.0;
	double lon1 = referencePosition.longitude * 3.14159265358979323846 / 180.0;
	double lat2 = position.latitude * 3.14159265358979323846 / 180.0;
	double lon2 = position.longitude * 3.14159265358979323846 / 180.0;
	
	double dLon = lon2 - lon1;
	
	double y = sin(dLon) * cos(lat2);
	double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
	
	double bearing = atan2(y, x);
	return bearing * 180.0 / 3.14159265358979323846;  // 转换为度
}

// 位置积分函数：由速度增量推算新的经纬高
GeoPosition updateGeoPosition(const GeoPosition & pos, const Vector3 & vel, double dt) {
	GeoPosition newPos = pos;

	double dNorth = vel.north * dt;
	double dEast = vel.east * dt;

	// 纬度变化 = 北向距离 / 地球半径
	newPos.latitude += (dNorth / EARTH_RADIUS) * (180.0 / M_PI);

	// 经度变化 = 东向距离 / (地球半径 * cos(纬度))
	double radiusAtLat = EARTH_RADIUS * std::cos(pos.latitude * M_PI / 180.0);
	if (std::abs(radiusAtLat) > 1e-6)
		newPos.longitude += (dEast / radiusAtLat) * (180.0 / M_PI);

	// 高度变化
	newPos.altitude += vel.up * dt;

	return newPos;
}
