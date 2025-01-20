set(publicHeaderDir ${PROJECT_SOURCE_DIR}/include)

set(publicHeaders
        "kine/Kine.hpp"
        "kine/KineComponent.hpp"
        "kine/KineLimit.hpp"
        "kine/KineLink.hpp"

        "kine/ik/CCDSolver.hpp"
        "kine/ik/DNNSolver.hpp"
        "kine/ik/IKSolver.hpp"

        "kine/joints/KineJoint.hpp"
        "kine/joints/PrismaticJoint.hpp"
        "kine/joints/RevoluteJoint.hpp"

        "kine/math/Euler.hpp"
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
add_library(kine::kine ALIAS kine)
target_compile_features(kine PUBLIC "cxx_std_20")
target_include_directories(kine
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
        PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

if (DEFINED ENV{ONNX_RUNTIME_DIR})
    target_include_directories(kine PUBLIC "$ENV{ONNX_RUNTIME_DIR}/include")
    target_link_directories(kine PUBLIC "$ENV{ONNX_RUNTIME_DIR}/lib")

    if (WIN32)
        set(ONNX_RUNTIME_LIB onnxruntime.lib)
    else ()
        set(ONNX_RUNTIME_LIB onnxruntime.so)
    endif ()
    target_link_libraries(kine PRIVATE ${ONNX_RUNTIME_LIB})

    target_sources(kine PRIVATE "kine/ik/DNNSolver.cpp")
    target_compile_definitions(kine PUBLIC KINE_WITH_DNN)
endif ()
