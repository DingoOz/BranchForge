#pragma once

#include <string>
#include <memory>
#include <optional>

namespace MyProject {

namespace MyProject {

// Blackboard data types for MyBehaviorTree
struct BlackboardData {
    // Common robot data types
    struct Position {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
    
    struct Orientation {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };
    
    struct Pose {
        Position position;
        Orientation orientation;
    };
    
    // Tree-specific data types
};

} // namespace MyProject

} // namespace MyProject
