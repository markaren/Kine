set(publicHeaderDir ${PROJECT_SOURCE_DIR}/include)

set(publicHeaders
        "kine/Kine.hpp"
        "kine/Kine.hpp"
        "kine/KineComponent.hpp"
        "kine/KineLimit.hpp"
        "kine/KineLink.hpp"

        "kine/ik/CCDSolver.hpp"
        "kine/ik/IKSolver.hpp"

        "kine/joints/KineJoint.hpp"
        "kine/joints/PrismaticJoint.hpp"
        "kine/joints/RevoluteJoint.hpp"

        "kine/math/MathUtils.hpp"
        "kine/math/Matrix4.hpp"
        "kine/math/Quaternion.hpp"
        "kine/math/Vector3.hpp"
)

set(publicHeadersFull)
foreach (header IN LISTS publicHeaders)
    list(APPEND publicHeadersFull "${publicHeaderDir}/${header}")
endforeach ()

set(sources
        "kine/math/Euler.cpp"
        "kine/math/MathUtils.cpp"
        "kine/math/Matrix4.cpp"
        "kine/math/Quaternion.cpp"
        "kine/math/Vector3.cpp"
)


add_library(kine ${sources} ${publicHeadersFull})
target_include_directories(kine PUBLIC "${PROJECT_SOURCE_DIR}/include")
