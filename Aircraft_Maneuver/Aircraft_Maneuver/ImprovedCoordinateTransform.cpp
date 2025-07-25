#include "ImprovedCoordinateTransform.h"
#include <cmath>

// 静态常量定义
const double ImprovedCoordinateTransform::EARTH_RADIUS_EQUATOR = 6378137.0;
const double ImprovedCoordinateTransform::EARTH_RADIUS_POLAR = 6356752.314245;
const double ImprovedCoordinateTransform::EARTH_FLATTENING = 1.0 / 298.257223563;
const double ImprovedCoordinateTransform::EARTH_ECCENTRICITY_SQ = 0.006694379990141316;
const double ImprovedCoordinateTransform::PI = 3.14159265358979323846;

GeoPosition ImprovedCoordinateTransform::updateGeoPositionImproved(const GeoPosition& pos, const Vector3& velocity, double dt) {
    GeoPosition newPos = pos;
    
    // 计算当前纬度下的地球半径
    double radius = calculateEarthRadius(pos.latitude);
    
    // 计算北向和东向的距离变化
    double dNorth = velocity.north * dt;
    double dEast = velocity.east * dt;
    
    // 纬度变化：考虑地球曲率
    double latChange = (dNorth / radius) * radToDeg(1.0);
    newPos.latitude += latChange;
    
    // 经度变化：考虑纬度对经度间距的影响
    double radiusAtLat = calculatePrimeVerticalRadius(pos.latitude) * cos(degToRad(pos.latitude));
    if (std::abs(radiusAtLat) > 1e-6) {
        double lonChange = (dEast / radiusAtLat) * radToDeg(1.0);
        newPos.longitude += lonChange;
    }
    
    // 高度变化
    newPos.altitude += velocity.up * dt;
    
    return newPos;
}

double ImprovedCoordinateTransform::calculateDistanceImproved(const GeoPosition& pos1, const GeoPosition& pos2) {
    double lat1 = degToRad(pos1.latitude);
    double lon1 = degToRad(pos1.longitude);
    double lat2 = degToRad(pos2.latitude);
    double lon2 = degToRad(pos2.longitude);
    
    // 使用Vincenty公式计算椭球体上的距离
    double L = lon2 - lon1;
    double tanU1 = (1 - EARTH_FLATTENING) * tan(lat1);
    double tanU2 = (1 - EARTH_FLATTENING) * tan(lat2);
    double U1 = atan(tanU1);
    double U2 = atan(tanU2);
    
    double lambda = L;
    double lambdaP;
    int iterations = 0;
    
    // 声明在循环外部使用的变量
    double sinU1, cosU1, sinU2, cosU2, cosSqAlpha;
    
    do {
        sinU1 = sin(U1);
        cosU1 = cos(U1);
        sinU2 = sin(U2);
        cosU2 = cos(U2);
        double sinLambda = sin(lambda);
        double cosLambda = cos(lambda);
        
        double sinSigma = sqrt((cosU2 * sinLambda) * (cosU2 * sinLambda) +
                              (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda) * 
                              (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda));
        
        if (sinSigma == 0) return 0;
        
        double cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
        double sigma = atan2(sinSigma, cosSigma);
        double sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
        cosSqAlpha = 1 - sinAlpha * sinAlpha;
        double cos2SigmaM = cosSigma - 2 * sinU1 * sinU2 / cosSqAlpha;
        
        double C = EARTH_FLATTENING / 16 * cosSqAlpha * (4 + EARTH_FLATTENING * (4 - 3 * cosSqAlpha));
        lambdaP = lambda;
        lambda = L + (1 - C) * EARTH_FLATTENING * sinAlpha * 
                (sigma + C * sinSigma * (cos2SigmaM + C * cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM)));
        
    } while (std::abs(lambda - lambdaP) > 1e-12 && ++iterations < 100);
    
    double uSq = cosSqAlpha * (EARTH_RADIUS_EQUATOR * EARTH_RADIUS_EQUATOR - EARTH_RADIUS_POLAR * EARTH_RADIUS_POLAR) / 
                 (EARTH_RADIUS_POLAR * EARTH_RADIUS_POLAR);
    double A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
    double B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));
    
    double sinSigma = sqrt((cosU2 * sin(lambda)) * (cosU2 * sin(lambda)) +
                          (cosU1 * sinU2 - sinU1 * cosU2 * cos(lambda)) * 
                          (cosU1 * sinU2 - sinU1 * cosU2 * cos(lambda)));
    double cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cos(lambda);
    double sigma = atan2(sinSigma, cosSigma);
    double cos2SigmaM = cosSigma - 2 * sinU1 * sinU2 / cosSqAlpha;
    
    double s = EARTH_RADIUS_POLAR * A * (sigma - B * sinSigma * (cos2SigmaM + B / 4 * cosSigma * 
                (-1 + 2 * cos2SigmaM * cos2SigmaM) - B / 6 * cos2SigmaM * (-3 + 4 * sinSigma * sinSigma) * 
                (-3 + 4 * cos2SigmaM * cos2SigmaM)));
    
    return s;
}

double ImprovedCoordinateTransform::calculateBearing(const GeoPosition& from, const GeoPosition& to) {
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

double ImprovedCoordinateTransform::calculateInitialBearing(const GeoPosition& from, const GeoPosition& to) {
    double lat1 = degToRad(from.latitude);
    double lon1 = degToRad(from.longitude);
    double lat2 = degToRad(to.latitude);
    double lon2 = degToRad(to.longitude);
    
    double dLon = lon2 - lon1;
    
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    
    double bearing = atan2(y, x);
    return fmod(radToDeg(bearing) + 360.0, 360.0);
}

GeoPosition ImprovedCoordinateTransform::calculateDestination(const GeoPosition& from, double distance, double bearing) {
    double lat1 = degToRad(from.latitude);
    double lon1 = degToRad(from.longitude);
    double brng = degToRad(bearing);
    
    // 使用平均地球半径
    double R = calculateEarthRadius(from.latitude);
    double angularDistance = distance / R;
    
    double lat2 = asin(sin(lat1) * cos(angularDistance) + cos(lat1) * sin(angularDistance) * cos(brng));
    double lon2 = lon1 + atan2(sin(brng) * sin(angularDistance) * cos(lat1),
                               cos(angularDistance) - sin(lat1) * sin(lat2));
    
    GeoPosition destination;
    destination.latitude = radToDeg(lat2);
    destination.longitude = radToDeg(lon2);
    destination.altitude = from.altitude;  // 保持相同高度
    
    return destination;
}

GeoPosition ImprovedCoordinateTransform::calculateMidpoint(const GeoPosition& pos1, const GeoPosition& pos2) {
    double lat1 = degToRad(pos1.latitude);
    double lon1 = degToRad(pos1.longitude);
    double lat2 = degToRad(pos2.latitude);
    double lon2 = degToRad(pos2.longitude);
    
    double Bx = cos(lat2) * cos(lon2 - lon1);
    double By = cos(lat2) * sin(lon2 - lon1);
    
    double latMid = atan2(sin(lat1) + sin(lat2),
                         sqrt((cos(lat1) + Bx) * (cos(lat1) + Bx) + By * By));
    double lonMid = lon1 + atan2(By, cos(lat1) + Bx);
    
    GeoPosition midpoint;
    midpoint.latitude = radToDeg(latMid);
    midpoint.longitude = radToDeg(lonMid);
    midpoint.altitude = (pos1.altitude + pos2.altitude) / 2.0;
    
    return midpoint;
}

bool ImprovedCoordinateTransform::isSameHemisphere(const GeoPosition& pos1, const GeoPosition& pos2) {
    // 检查是否在同一半球（纬度符号相同）
    return (pos1.latitude >= 0 && pos2.latitude >= 0) || (pos1.latitude < 0 && pos2.latitude < 0);
}

double ImprovedCoordinateTransform::calculateMeridianRadius(double latitude) {
    double lat = degToRad(latitude);
    double sinLat = sin(lat);
    double e2 = EARTH_ECCENTRICITY_SQ;
    
    return EARTH_RADIUS_EQUATOR * (1 - e2) / pow(1 - e2 * sinLat * sinLat, 1.5);
}

double ImprovedCoordinateTransform::calculatePrimeVerticalRadius(double latitude) {
    double lat = degToRad(latitude);
    double sinLat = sin(lat);
    double e2 = EARTH_ECCENTRICITY_SQ;
    
    return EARTH_RADIUS_EQUATOR / sqrt(1 - e2 * sinLat * sinLat);
}

double ImprovedCoordinateTransform::calculateEarthRadius(double latitude) {
    double lat = degToRad(latitude);
    double sinLat = sin(lat);
    double cosLat = cos(lat);
    double e2 = EARTH_ECCENTRICITY_SQ;
    
    // 计算平均曲率半径
    double meridianRadius = EARTH_RADIUS_EQUATOR * (1 - e2) / pow(1 - e2 * sinLat * sinLat, 1.5);
    double primeVerticalRadius = EARTH_RADIUS_EQUATOR / sqrt(1 - e2 * sinLat * sinLat);
    
    return sqrt(meridianRadius * primeVerticalRadius);
} 