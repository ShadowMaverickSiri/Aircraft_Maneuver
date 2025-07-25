#ifndef FIGHTER_JET_H
#define FIGHTER_JET_H

#include "AircraftModelLibrary.h"
#include <string>

// 战斗机类：继承自 Aircraft
class FighterJet : public Aircraft {
public:
    FighterJet(const std::string& modelName)
        : Aircraft(modelName) {
        if (modelName == "F-15") {
            maxThrust = 200000.0;
            dragCoeff = 0.02;
        } else if (modelName == "Su-27") {
            maxThrust = 220000.0;
            dragCoeff = 0.025;
        }
        // ... 其它型号
    }
    Vector3 computeAcceleration() const override;

private:
	double maxThrust;   // 最大推力 (牛顿)
	double dragCoeff;   // 阻力系数
};

#endif // FIGHTER_JET_H
