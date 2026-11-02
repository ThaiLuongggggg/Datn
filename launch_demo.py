#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Trình khởi chạy trực tiếp Launch Description cho Nav2 A* Demo.
Cho phép khởi chạy đầy đủ các node (map_server, planner_server, lifecycle_manager, rviz2)
ngay cả khi chưa cài đặt gói 'ros-humble-ros2launch'.
Hỗ trợ truyền đối số dòng lệnh dạng: arg:=value (ví dụ: map:=... planner_plugin:=...)
"""

import sys
import os
from launch import LaunchService, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    launch_file = os.path.join(script_dir, 'launch', 'astar_demo.launch.py')

    if not os.path.isfile(launch_file):
        print(f"LỖI: Không tìm thấy file launch tại: {launch_file}", file=sys.stderr)
        return 1

    # Tách các đối số dạng key:=value
    launch_args = {}
    pass_through_argv = []
    for arg in sys.argv[1:]:
        if ':=' in arg:
            k, v = arg.split(':=', 1)
            launch_args[k.strip()] = v.strip()
        else:
            pass_through_argv.append(arg)

    # Đóng gói qua IncludeLaunchDescription để truyền tham số chính xác
    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments=launch_args.items()
        )
    ])

    ls = LaunchService(argv=pass_through_argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    sys.exit(main())
