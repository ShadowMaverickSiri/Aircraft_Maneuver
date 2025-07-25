cmake_minimum_required(VERSION 3.10)
project(Aircraft_Maneuver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== Eigen库查找策略 =====
# 策略1: 尝试使用find_package查找Eigen3
find_package(Eigen3 QUIET)

# 策略2: 如果没有找到Eigen3，尝试查找Eigen
if(NOT TARGET Eigen3::Eigen)
    find_package(Eigen QUIET)
endif()

# 策略3: 如果仍然没有找到，尝试使用pkg-config
if(NOT TARGET Eigen3::Eigen AND NOT TARGET Eigen::Eigen)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(EIGEN3 QUIET eigen3)
        if(EIGEN3_FOUND)
            include_directories(${EIGEN3_INCLUDE_DIRS})
        endif()
    endif()
endif()

# 策略4: 如果以上都没有找到，尝试常见的包含路径
if(NOT TARGET Eigen3::Eigen AND NOT TARGET Eigen::Eigen AND NOT EIGEN3_FOUND)
    # 检查常见的Eigen安装路径
    set(EIGEN_POSSIBLE_PATHS
        "/usr/include/eigen3"
        "/usr/local/include/eigen3"
        "/opt/local/include/eigen3"
        "/usr/include"
        "/usr/local/include"
    )
    
    foreach(PATH ${EIGEN_POSSIBLE_PATHS})
        if(EXISTS "${PATH}/Eigen/Dense")
            include_directories(${PATH})
            set(EIGEN_FOUND_MANUALLY TRUE)
            message(STATUS "Found Eigen manually at: ${PATH}")
            break()
        endif()
    endforeach()
endif()

# 检查是否找到Eigen库
if(TARGET Eigen3::Eigen OR TARGET Eigen::Eigen OR EIGEN3_FOUND OR EIGEN_FOUND_MANUALLY)
    message(STATUS "Eigen library found - coordinate transform features will be available")
    add_definitions(-DUSE_EIGEN)
    set(EIGEN_AVAILABLE TRUE)
else()
    message(STATUS "Eigen library not found - coordinate transform features will be disabled")
    message(STATUS "To enable Eigen features, install Eigen3 library")
    set(EIGEN_AVAILABLE FALSE)
endif()

# ===== 源文件定义 =====
# 基础源文件（不依赖Eigen）
set(BASE_SOURCES
    main.cpp
    AircraftModelLibrary.cpp
    FighterJet.cpp
    ManeuverModel.cpp
    EulerAngleCalculation.cpp
    CoordinateTransform.cpp
    ImprovedCoordinateTransform.cpp
)

# 创建主可执行文件
add_executable(Aircraft_Maneuver
    main.cpp
    AircraftModelLibrary.cpp
    FighterJet.cpp
    ManeuverModel.cpp
    EulerAngleCalculation.cpp
    CoordinateTransform.cpp
    ImprovedCoordinateTransform.cpp
)

# 链接Eigen库（如果可用）
if(EIGEN_AVAILABLE)
    if(TARGET Eigen3::Eigen)
        target_link_libraries(Aircraft_Maneuver Eigen3::Eigen)
    elseif(TARGET Eigen::Eigen)
        target_link_libraries(Aircraft_Maneuver Eigen::Eigen)
    elseif(EIGEN3_FOUND)
        target_link_libraries(Aircraft_Maneuver ${EIGEN3_LIBRARIES})
    endif()
endif()

# 编译选项
target_compile_options(Aircraft_Maneuver PRIVATE -Wall -Wextra)

# 如果使用MSVC编译器
if(MSVC)
    target_compile_options(Aircraft_Maneuver PRIVATE /W4)
endif()

# ===== 机动模型示例 =====
# 删除已删除/重命名的文件相关行

# ===== 测试程序 =====
add_executable(test_aircraft_basic tests/test_aircraft_basic.cpp
    AircraftModelLibrary.cpp FighterJet.cpp ManeuverModel.cpp EulerAngleCalculation.cpp)
add_executable(test_coordinate_transform tests/test_coordinate_transform.cpp
    AircraftModelLibrary.cpp FighterJet.cpp CoordinateTransform.cpp ImprovedCoordinateTransform.cpp EulerAngleCalculation.cpp)
add_executable(test_compile tests/test_compile.cpp
    AircraftModelLibrary.cpp FighterJet.cpp ManeuverModel.cpp EulerAngleCalculation.cpp)

# 链接Eigen库（如果可用）
if(EIGEN_AVAILABLE)
    if(TARGET Eigen3::Eigen)
        target_link_libraries(test_compile Eigen3::Eigen)
    elseif(TARGET Eigen::Eigen)
        target_link_libraries(test_compile Eigen::Eigen)
    elseif(EIGEN3_FOUND)
        target_link_libraries(test_compile ${EIGEN3_LIBRARIES})
    endif()
endif()

# 编译选项
target_compile_options(test_compile PRIVATE -Wall -Wextra)
if(MSVC)
    target_compile_options(test_compile PRIVATE /W4)
endif()

# ===== 坐标转换测试程序 =====
# 编译选项
target_compile_options(test_coordinate_transform PRIVATE -Wall -Wextra)
if(MSVC)
    target_compile_options(test_coordinate_transform PRIVATE /W4)
endif()

# ===== 坐标转换示例（条件编译） =====
# 检查坐标转换相关文件是否存在
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/CoordinateTransform.h" AND 
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/CoordinateTransform.cpp" AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/CoordinateTransformExample.cpp")
    
    add_executable(CoordinateTransformExample 
        CoordinateTransformExample.cpp 
        CoordinateTransform.cpp 
        AircraftModelLibrary.cpp
        EulerAngleCalculation.cpp
    )
    
    # 链接Eigen库（如果可用）
    if(EIGEN_AVAILABLE)
        if(TARGET Eigen3::Eigen)
            target_link_libraries(CoordinateTransformExample Eigen3::Eigen)
        elseif(TARGET Eigen::Eigen)
            target_link_libraries(CoordinateTransformExample Eigen::Eigen)
        elseif(EIGEN3_FOUND)
            target_link_libraries(CoordinateTransformExample ${EIGEN3_LIBRARIES})
        endif()
    endif()
    
    # 编译选项
    target_compile_options(CoordinateTransformExample PRIVATE -Wall -Wextra)
    if(MSVC)
        target_compile_options(CoordinateTransformExample PRIVATE /W4)
    endif()
    
    message(STATUS "Coordinate transform example will be built")
else()
    message(STATUS "Coordinate transform files not found - skipping coordinate transform example")
endif()

# ===== 坐标转换演示程序 =====
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/CoordinateTransformDemo.cpp")
    add_executable(CoordinateTransformDemo 
        CoordinateTransformDemo.cpp 
        AircraftModelLibrary.cpp 
        FighterJet.cpp
        ImprovedCoordinateTransform.cpp
        EulerAngleCalculation.cpp
    )
    
    # 编译选项
    target_compile_options(CoordinateTransformDemo PRIVATE -Wall -Wextra)
    if(MSVC)
        target_compile_options(CoordinateTransformDemo PRIVATE /W4)
    endif()
    
    message(STATUS "Coordinate transform demo will be built")
else()
    message(STATUS "Coordinate transform demo file not found - skipping coordinate transform demo")
endif()

# ===== 示例/演示 =====
add_executable(example_maneuver_usage examples/example_maneuver_usage.cpp
    AircraftModelLibrary.cpp FighterJet.cpp ManeuverModel.cpp EulerAngleCalculation.cpp)

# ===== 安装配置 =====
# 安装可执行文件
install(TARGETS Aircraft_Maneuver ManeuverModelExample test_compile
        RUNTIME DESTINATION bin)

# 如果坐标转换示例存在，也安装它
if(TARGET CoordinateTransformExample)
    install(TARGETS CoordinateTransformExample
            RUNTIME DESTINATION bin)
endif()

# 安装头文件
install(FILES 
    AircraftModelLibrary.h
    FighterJet.h
    Maneuvers.h
    ManeuverModels.h
    EulerAngleCalculation.h
    DESTINATION include)

# 如果坐标转换头文件存在，也安装它
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/CoordinateTransform.h")
    install(FILES CoordinateTransform.h
            DESTINATION include)
endif()

# ===== 配置信息输出 =====
message(STATUS "=== Aircraft Maneuver Configuration ===")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Eigen Library: ${EIGEN_AVAILABLE}")
if(EIGEN_AVAILABLE)
    if(TARGET Eigen3::Eigen)
        message(STATUS "Eigen Target: Eigen3::Eigen")
    elseif(TARGET Eigen::Eigen)
        message(STATUS "Eigen Target: Eigen::Eigen")
    elseif(EIGEN3_FOUND)
        message(STATUS "Eigen Target: pkg-config")
    else()
        message(STATUS "Eigen Target: manual path")
    endif()
endif()
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "=====================================") 