#include "CoordinateTransform.h"
#include <cmath>

// 静态常量定义
const double CoordinateTransform::EARTH_RADIUS = 6371000.0;
const double CoordinateTransform::PI = 3.14159265358979323846;

// 辅助函数实现
double CoordinateTransform::degToRad(double degrees) {
    return degrees * PI / 180.0;
}

double CoordinateTransform::radToDeg(double radians) {
    return radians * 180.0 / PI;
}

#ifdef USE_EIGEN

Eigen::Vector3d CoordinateTransform::geodeticToECEF(const GeoPosition& geodetic) {
    double lat = degToRad(geodetic.latitude);
    double lon = degToRad(geodetic.longitude);
    double h = geodetic.altitude;
    
    // 地球椭球体参数（WGS84）
    const double a = 6378137.0;  // 长半轴
    const double e2 = 0.006694379990141316;  // 第一偏心率平方
    
    double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));  // 卯酉圈曲率半径
    
    Eigen::Vector3d ecef;
    ecef.x() = (N + h) * cos(lat) * cos(lon);
    ecef.y() = (N + h) * cos(lat) * sin(lon);
    ecef.z() = (N * (1 - e2) + h) * sin(lat);
    
    return ecef;
}

GeoPosition CoordinateTransform::ecefToGeodetic(const Eigen::Vector3d& ecef) {
    const double a = 6378137.0;  // 长半轴
    const double e2 = 0.006694379990141316;  // 第一偏心率平方
    const double b = 6356752.314245;  // 短半轴
    
    double x = ecef.x();
    double y = ecef.y();
    double z = ecef.z();
    
    double p = sqrt(x * x + y * y);
    double lat = atan2(z, p * (1 - e2));
    
    // 迭代求解纬度
    for (int i = 0; i < 5; ++i) {
        double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));
        double h = p / cos(lat) - N;
        double lat_new = atan2(z, p * (1 - e2 * N / (N + h)));
        if (abs(lat - lat_new) < 1e-12) break;
        lat = lat_new;
    }
    
    double lon = atan2(y, x);
    double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));
    double h = p / cos(lat) - N;
    
    GeoPosition geodetic;
    geodetic.latitude = radToDeg(lat);
    geodetic.longitude = radToDeg(lon);
    geodetic.altitude = h;
    
    return geodetic;
}

Eigen::Matrix3d CoordinateTransform::getECEFToNUERotation(const GeoPosition& position) {
    double lat = degToRad(position.latitude);
    double lon = degToRad(position.longitude);
    
    // 构建旋转矩阵：从ECEF到NUE
    Eigen::Matrix3d R;
    R << -sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat),
         cos(lat) * cos(lon),  cos(lat) * sin(lon),  sin(lat),
         -sin(lon),            cos(lon),             0;
    
    return R;
}

Eigen::Matrix3d CoordinateTransform::getNUEToECEFRotation(const GeoPosition& position) {
    // NUE到ECEF的旋转矩阵是ECEF到NUE的转置
    return getECEFToNUERotation(position).transpose();
}

Eigen::Vector3d CoordinateTransform::nueToECEFVelocity(const Vector3& nueVel, const GeoPosition& position) {
    Eigen::Matrix3d R = getNUEToECEFRotation(position);
    Eigen::Vector3d nueVec(nueVel.north, nueVel.up, nueVel.east);
    return R * nueVec;
}

Vector3 CoordinateTransform::ecefToNUEVelocity(const Eigen::Vector3d& ecefVel, const GeoPosition& position) {
    Eigen::Matrix3d R = getECEFToNUERotation(position);
    Eigen::Vector3d nueVec = R * ecefVel;
    
    Vector3 nueVel;
    nueVel.north = nueVec.x();
    nueVel.up = nueVec.y();
    nueVel.east = nueVec.z();
    
    return nueVel;
}

Vector3 CoordinateTransform::ecefToNUEPosition(const Eigen::Vector3d& ecefPos, const GeoPosition& referencePosition) {
    // 将ECEF位置向量转换为相对于参考点的NUE位置向量
    Eigen::Vector3d referenceECEF = geodeticToECEF(referencePosition);
    Eigen::Vector3d relativeECEF = ecefPos - referenceECEF;
    
    // 使用旋转矩阵将相对ECEF向量转换为NUE向量
    Eigen::Matrix3d R = getECEFToNUERotation(referencePosition);
    Eigen::Vector3d nueVec = R * relativeECEF;
    
    Vector3 nuePos;
    nuePos.north = nueVec.x();
    nuePos.up = nueVec.y();
    nuePos.east = nueVec.z();
    
    return nuePos;
}

GeoPosition CoordinateTransform::updateGeoPositionEigen(const GeoPosition& pos, const Vector3& velocity, double dt) {
    // 1. 将当前位置转换为ECEF坐标
    Eigen::Vector3d ecefPos = geodeticToECEF(pos);
    
    // 2. 将NUE速度转换为ECEF速度
    Eigen::Vector3d ecefVel = nueToECEFVelocity(velocity, pos);
    
    // 3. 在ECEF坐标系下更新位置
    Eigen::Vector3d newEcefPos = ecefPos + ecefVel * dt;
    
    // 4. 将新的ECEF位置转换回地理坐标
    return ecefToGeodetic(newEcefPos);
}

#endif // USE_EIGEN

double CoordinateTransform::calculateDistance(const GeoPosition& pos1, const GeoPosition& pos2) {
    double lat1 = degToRad(pos1.latitude);
    double lon1 = degToRad(pos1.longitude);
    double lat2 = degToRad(pos2.latitude);
    double lon2 = degToRad(pos2.longitude);
    
    // 使用Haversine公式计算球面距离
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    
    double a = sin(dLat/2) * sin(dLat/2) + 
               cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return EARTH_RADIUS * c;
}

double CoordinateTransform::calculateBearing(const GeoPosition& from, const GeoPosition& to) {
    double lat1 = degToRad(from.latitude);
    double lon1 = degToRad(from.longitude);
    double lat2 = degToRad(to.latitude);
    double lon2 = degToRad(to.longitude);
    
    double dLon = lon2 - lon1;
    
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    
    double bearing = atan2(y, x);
    return radToDeg(bearing);
} 