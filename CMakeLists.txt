cmake_minimum_required(VERSION 3.5)
project(ur_rtde_examples)

set(ur_rtde_DIR "/usr/local/lib/cmake/ur_rtde/")
find_package(ur_rtde REQUIRED )
###################################
set(List_official 
forcemode_example
move_until_contact 
io_example 
move_path_async_example 
movej_path_with_blend
servoj_example 
speedj_example)  
move_async_example
robotiq_gripper_example)
##################################
set(List_own moving_fluid)
##################################

set( LIst ${List_official} ${List_own})

foreach(list  ${List}  )

add_executable(${list} ${list}.cpp)
target_link_libraries(${list}  PUBLIC ur_rtde::rtde)

endforeach()

