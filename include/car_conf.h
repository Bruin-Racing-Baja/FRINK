#ifndef CAR_CONF_H
#define CAR_CONF_H
constexpr float ENGINE_COUNTS_PER_ROT = 16; // [ engine_count / rot]
constexpr float GEAR_COUNTS_PER_ROT = 6; // [ gear_count / rot ]
constexpr float GEAR_TO_WHEEL_RATIO = 1.0; // [ gear_rot / wheel_rot ]
constexpr float SECONDARY_TO_WHEEL_RATIO = (46.0/17.0) * (56.0/19.0); // ~7.975 [ secondary_rot / wheel_rot ] 
constexpr float WHEEL_TO_SECONDARY_RATIO =  1.0 / SECONDARY_TO_WHEEL_RATIO; // ~0.1253 [ wheel
#endif