#ifndef IMPROVED_COORDINATE_TRANSFORM_H
#define IMPROVED_COORDINATE_TRANSFORM_H

#include "AircraftModelLibrary.h"
#include <cmath>

class ImprovedCoordinateTransform {
public:
    // 地球参数 (WGS84)
    static const double EARTH_RADIUS_EQUATOR;  // 赤道半径
    static const double EARTH_RADIUS_POLAR;  // 极半径
    static const double EARTH_FLATTENING;  // 扁率
    static const double EARTH_ECCENTRICITY_SQ;  // 第一偏心率平方
    
    static const double PI;  // 圆周率

    // 改进的位置更新函数
    static GeoPosition updateGeoPositionImproved(const GeoPosition& pos, const Vector3& velocity, double dt);
    
    // 计算两点间的精确距离 (使用椭球体模型)
    static double calculateDistanceImproved(const GeoPosition& pos1, const GeoPosition& pos2);
    
    // 计算方位角
    static double calculateBearing(const GeoPosition& from, const GeoPosition& to);
    
    // 计算初始方位角 (大圆航线)
    static double calculateInitialBearing(const GeoPosition& from, const GeoPosition& to);
    
    // 计算目标点 (给定起点、距离和方位角)
    static GeoPosition calculateDestination(const GeoPosition& from, double distance, double bearing);
    
    // 计算中点
    static GeoPosition calculateMidpoint(const GeoPosition& pos1, const GeoPosition& pos2);
    
    // 检查两点是否在同一半球
    static bool isSameHemisphere(const GeoPosition& pos1, const GeoPosition& pos2);

     // 计算地球曲率半径 (平均)
    static double calculateEarthRadius(double latitude);

private:
    // 辅助函数：度转弧度
    static double degToRad(double degrees) { return degrees * PI / 180.0; }
    
    // 辅助函数：弧度转度
    static double radToDeg(double radians) { return radians * 180.0 / PI; }
    
    // 计算卯酉圈曲率半径
    static double calculateMeridianRadius(double latitude);
    
    // 计算卯酉圈曲率半径
    static double calculatePrimeVerticalRadius(double latitude);
    
   
};

#endif // IMPROVED_COORDINATE_TRANSFORM_H 