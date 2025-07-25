#include <string>
#include "FighterJet.h"
#include <cmath>

FighterJet::FighterJet(const std::string& modelName)
	: Aircraft(modelName),
	maxThrust(200000.0),
	dragCoeff(0.02)
{}

// 计算加速度
Vector3 FighterJet::computeAcceleration() const {
	double vx = velocity.north;
	double vy = velocity.up;
	double vz = velocity.east;
	double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

	Vector3 acc{ 0.0, 0.0, 0.0 };
	if (speed > 1e-3) {
		double thrustAcc = maxThrust / performance.mass;
		double dragAcc = dragCoeff * speed * speed / performance.mass;
		double netAcc = thrustAcc - dragAcc;

		// 加速度方向与当前速度方向一致
		acc.north = netAcc * (vx / speed);
		acc.up = netAcc * (vy / speed);
		acc.east = netAcc * (vz / speed);
	}
	return acc;
}
