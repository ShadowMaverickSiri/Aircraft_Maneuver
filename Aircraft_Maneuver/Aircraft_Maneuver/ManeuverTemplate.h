#ifndef MANEUVER_TEMPLATE_H
#define MANEUVER_TEMPLATE_H

#include "AircraftModelLibrary.h"
#include <vector>
#include <functional>

// 机动点结构体
struct ManeuverPoint {
    double time;           // 时间点
    double turnRate;       // 转向速率
    double climbRate;      // 爬升速率
    double rollRate;       // 滚转速率
    double pitchRate;      // 俯仰速率
    double yawRate;        // 偏航速率
    
    ManeuverPoint(double t, double tr, double cr, double rr = 0.0, 
                  double pr = 0.0, double yr = 0.0)
        : time(t), turnRate(tr), climbRate(cr), rollRate(rr), 
          pitchRate(pr), yawRate(yr) {}
};

// 通用机动模板类
class ManeuverTemplate {
public:
    // 构造函数
    ManeuverTemplate(const std::string& name, const std::vector<ManeuverPoint>& points);
    
    // 获取机动名称
    std::string getName() const { return name; }
    
    // 获取机动点序列
    const std::vector<ManeuverPoint>& getPoints() const { return points; }
    
    // 计算指定时间的机动参数
    ManeuverParameters interpolateParameters(double currentTime) const;
    
    // 检查机动是否完成
    bool isCompleted(double currentTime) const;
    
    // 获取总持续时间
    double getDuration() const;

private:
    std::string name;
    std::vector<ManeuverPoint> points;
};

// 预定义机动模板
class ManeuverTemplates {
public:
    // 创建筋斗机动模板
    static ManeuverTemplate createLoopManeuver();
    
    // 创建横滚机动模板
    static ManeuverTemplate createRollManeuver();
    
    // 创建S形机动模板
    static ManeuverTemplate createSManeuver();
    
    // 创建蛇形机动模板
    static ManeuverTemplate createSnakeManeuver();
    
    // 创建眼镜蛇机动模板
    static ManeuverTemplate createCobraManeuver();
    
    // 创建剪刀机动模板
    static ManeuverTemplate createScissorsManeuver();
    
    // 创建桶滚机动模板
    static ManeuverTemplate createBarrelRollManeuver();
    
    // 创建高G转弯模板
    static ManeuverTemplate createHighGTurnManeuver();
    
    // 创建垂直爬升模板
    static ManeuverTemplate createVerticalClimbManeuver();
    
    // 创建俯冲攻击模板
    static ManeuverTemplate createDiveAttackManeuver();
};

// 机动模板管理器
class ManeuverTemplateManager {
public:
    // 注册机动模板
    static void registerTemplate(const ManeuverTemplate& template_);
    
    // 获取机动模板
    static ManeuverTemplate getTemplate(const std::string& name);
    
    // 获取所有可用模板名称
    static std::vector<std::string> getAvailableTemplates();
    
    // 创建机动模型
    static std::shared_ptr<ManeuverModel> createManeuverModel(const std::string& templateName);

private:
    static std::map<std::string, ManeuverTemplate> templates;
};

#endif // MANEUVER_TEMPLATE_H 