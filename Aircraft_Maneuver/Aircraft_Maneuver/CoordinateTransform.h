#ifndef COORDINATE_TRANSFORM_H
#define COORDINATE_TRANSFORM_H

#include "AircraftModelLibrary.h"

#ifdef USE_EIGEN
#include <Eigen/Dense>
#endif

class CoordinateTransform {
public:
    // 地球参数
    static const double EARTH_RADIUS;  // 地球半径 (米)
    static const double PI;  // 圆周率

#ifdef USE_EIGEN
    // 将地理坐标转换为ECEF (Earth-Centered, Earth-Fixed) 坐标
    static Eigen::Vector3d geodeticToECEF(const GeoPosition& geodetic);
    
    // 将ECEF坐标转换为地理坐标
    static GeoPosition ecefToGeodetic(const Eigen::Vector3d& ecef);
    
    // 将NUE (North-Up-East) 速度向量转换为ECEF坐标系下的速度
    static Eigen::Vector3d nueToECEFVelocity(const Vector3& nueVel, const GeoPosition& position);
    
    // 将ECEF坐标系下的速度转换为NUE速度向量
    static Vector3 ecefToNUEVelocity(const Eigen::Vector3d& ecefVel, const GeoPosition& position);
    
    // 将ECEF位置向量转换为NUE位置向量
    static Vector3 ecefToNUEPosition(const Eigen::Vector3d& ecefPos, const GeoPosition& referencePosition);
    
    // 获取从ECEF到NUE的旋转矩阵
    static Eigen::Matrix3d getECEFToNUERotation(const GeoPosition& position);
    
    // 获取从NUE到ECEF的旋转矩阵
    static Eigen::Matrix3d getNUEToECEFRotation(const GeoPosition& position);
    
    // 更新地理位置的完整方法（使用Eigen）
    static GeoPosition updateGeoPositionEigen(const GeoPosition& pos, const Vector3& velocity, double dt);
#endif

    // 计算两点间的距离（使用Haversine公式）
    static double calculateDistance(const GeoPosition& pos1, const GeoPosition& pos2);
    
    // 计算方位角
    static double calculateBearing(const GeoPosition& from, const GeoPosition& to);

private:
    // 辅助函数：度转弧度
    static double degToRad(double degrees);
    
    // 辅助函数：弧度转度
    static double radToDeg(double radians);
    
    // 获取圆周率
    static double getPI() { return PI; }
};

#endif // COORDINATE_TRANSFORM_H 