cmake_minimum_required(VERSION 2.8)
project( Executable ) 
find_package( OpenCV REQUIRED )
add_executable( Executable QRCode_ZBar.cpp )
target_link_libraries( Executable ${OpenCV_LIBS} ${ZBAR_LIBRARIES} zbar)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")



