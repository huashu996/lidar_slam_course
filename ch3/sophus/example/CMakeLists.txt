find_package(Pangolin REQUIRED) #查找依赖包
include_directories(${Pangolin_INCLUDE_DIRS}) #用来找头文件的路径
add_executable(trajectoryError trajectoryError.cpp)
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES}) #将可执行文件与头文件链接

# Eigen
include_directories("/usr/local/include/eigen3")
target_link_libraries(trajectoryError ${Sophus_LIBRARIES} fmt)# trajectoryError依赖于Sophus，Sophus依赖于fmt

