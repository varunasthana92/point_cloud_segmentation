add_executable(segment main.cpp cloud.cpp voxel.cpp)
add_executable(trypcl trypcl.cpp)


include_directories(${PCL_INCLUDE_DIRS}
					${CMAKE_SOURCE_DIR}/include
					)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_include_directories(segment PUBLIC ${CMAKE_SOURCE_DIR}/include
						${OpenCV_INCLUDE_DIRS}
						${PCL_INCLUDE_DIRS}
						)

target_link_libraries (segment ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries (trypcl ${PCL_LIBRARIES})
