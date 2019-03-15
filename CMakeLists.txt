project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

include_directories(include/)
set(sources src/main.cpp src/io.cpp src/ukf.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  include_directories(/usr/local/include)
  include_directories(/usr/local/opt/openssl/include)
  link_directories(/usr/local/lib)
  link_directories(/usr/local/opt/openssl/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(UnscentedKF ${sources})
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  target_link_libraries(UnscentedKF z ssl uv uWS python2.7)
else()
  target_link_libraries(UnscentedKF z ssl uv uWS python)
endif()
