#include "AircraftModelLibrary.h"
#include "EulerAngleCalculation.h"
#include <cmath>
#include <stdexcept>
#include <sstream>

// ����뾶 (��λ����)
const double EARTH_RADIUS = 6371000.0;

// ��̬������ʼ��
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

	// �ٶȸ���
	velocity.north += a.north * dt;
	velocity.up += a.up * dt;
	velocity.east += a.east * dt;

	// ���ݵ�ǰ�ٶȸ���λ��
	position = updateGeoPosition(position, velocity, dt);
}

// ��ͳ�������������������ݣ�
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

// �µĻ���ģ�ͷ���
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
	// ���ٶ��������������̬��
	attitude = EulerAngleCalculator::calculateFromVelocity(velocity);
}

// ����ת����ط���ʵ��
void Aircraft::setReferencePosition(const GeoPosition& refPos) {
	referencePosition = refPos;
}

Vector3 Aircraft::getECEFPosition() const {
	// �򻯵�ECEF������㣨������Eigen��
	const double EARTH_RADIUS = 6371000.0;
	const double PI = 3.14159265358979323846;
	
	double lat = position.latitude * PI / 180.0;  // ת��Ϊ����
	double lon = position.longitude * PI / 180.0;
	double h = position.altitude;
	
	// �򻯵������������WGS84��
	const double a = 6378137.0;  // ������
	const double e2 = 0.006694379990141316;  // ��һƫ����ƽ��
	
	double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));  // î��Ȧ���ʰ뾶
	
	Vector3 ecef;
	ecef.north = (N + h) * cos(lat) * cos(lon);  // X����
	ecef.up = (N + h) * cos(lat) * sin(lon);     // Y����
	ecef.east = (N * (1 - e2) + h) * sin(lat);   // Z����
	
	return ecef;
}

Vector3 Aircraft::getLocalNUEPosition() const {
	// ��������ڲο����NUEλ��
	Vector3 refECEF = getECEFPosition();  // ��ǰ�ɻ�ECEFλ��
	
	// ����ο����ECEFλ��
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
	
	// �������λ��
	Vector3 relativeECEF;
	relativeECEF.north = refECEF.north - refECEFPos.north;
	relativeECEF.up = refECEF.up - refECEFPos.up;
	relativeECEF.east = refECEF.east - refECEFPos.east;
	
	// ת��ΪNUE����ϵ
	Vector3 localNUE;
	
	// ������ת���󣨴�ECEF��NUE��
	double sinLat = sin(refLat);
	double cosLat = cos(refLat);
	double sinLon = sin(refLon);
	double cosLon = cos(refLon);
	
	// ��ת����Ӧ��
	localNUE.north = -sinLat * cosLon * relativeECEF.north - sinLat * sinLon * relativeECEF.up + cosLat * relativeECEF.east;
	localNUE.up = cosLat * cosLon * relativeECEF.north + cosLat * sinLon * relativeECEF.up + sinLat * relativeECEF.east;
	localNUE.east = -sinLon * relativeECEF.north + cosLon * relativeECEF.up;
	
	return localNUE;
}

double Aircraft::getDistanceFromReference() const {
	// ���㵽�ο���ľ���
	double dLat = position.latitude - referencePosition.latitude;
	double dLon = position.longitude - referencePosition.longitude;
	double dAlt = position.altitude - referencePosition.altitude;
	
	// ת��Ϊ����
	const double PI = 3.14159265358979323846;
	dLat *= PI / 180.0;
	dLon *= PI / 180.0;
	
	// ʹ��Haversine��ʽ����ˮƽ����
	double lat1 = referencePosition.latitude * PI / 180.0;
	double lat2 = position.latitude * PI / 180.0;
	
	double a = sin(dLat/2) * sin(dLat/2) + cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	
	const double EARTH_RADIUS = 6371000.0;
	double horizontalDistance = EARTH_RADIUS * c;
	
	// �����ܾ��루�����߶ȣ�
	return sqrt(horizontalDistance * horizontalDistance + dAlt * dAlt);
}

double Aircraft::getBearingFromReference() const {
	// ����Ӳο��㵽��ǰλ�õķ�λ��
	double lat1 = referencePosition.latitude * 3.14159265358979323846 / 180.0;
	double lon1 = referencePosition.longitude * 3.14159265358979323846 / 180.0;
	double lat2 = position.latitude * 3.14159265358979323846 / 180.0;
	double lon2 = position.longitude * 3.14159265358979323846 / 180.0;
	
	double dLon = lon2 - lon1;
	
	double y = sin(dLon) * cos(lat2);
	double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
	
	double bearing = atan2(y, x);
	return bearing * 180.0 / 3.14159265358979323846;  // ת��Ϊ��
}

// λ�û��ֺ��������ٶ����������µľ�γ��
GeoPosition updateGeoPosition(const GeoPosition & pos, const Vector3 & vel, double dt) {
	GeoPosition newPos = pos;

	double dNorth = vel.north * dt;
	double dEast = vel.east * dt;

	// γ�ȱ仯 = ������� / ����뾶
	newPos.latitude += (dNorth / EARTH_RADIUS) * (180.0 / M_PI);

	// ���ȱ仯 = ������� / (����뾶 * cos(γ��))
	double radiusAtLat = EARTH_RADIUS * std::cos(pos.latitude * M_PI / 180.0);
	if (std::abs(radiusAtLat) > 1e-6)
		newPos.longitude += (dEast / radiusAtLat) * (180.0 / M_PI);

	// �߶ȱ仯
	newPos.altitude += vel.up * dt;

	return newPos;
}
