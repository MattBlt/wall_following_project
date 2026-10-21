cmake_minimum_required(VERSION 3.8)
project(limo_wf_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds urdf meshes
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
        scripts/twist_watchdog.py
        scripts/odom_noise.py
        scripts/square_drive_test.py
        scripts/export_bag.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
