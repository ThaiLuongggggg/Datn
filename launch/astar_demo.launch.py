import os
import sys
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # -------------------------------------------------------------------------
    # Xác định đường dẫn thư mục gốc của package
    # Hỗ trợ cả 2 trường hợp: Đã build bằng colcon hoặc chạy trực tiếp từ source
    # -------------------------------------------------------------------------
    try:
        pkg_dir = get_package_share_directory('nav2_astar_demo')
        is_installed = True
    except PackageNotFoundError:
        # Nếu chưa chạy colcon build / source install, dùng thư mục cha của launch
        pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        is_installed = False

    default_map_file = os.path.join(pkg_dir, 'maps', 'sample_map.yaml')
    default_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(pkg_dir, 'rviz', 'astar_demo.rviz')
    goal_bridge_script = os.path.join(pkg_dir, 'scripts', 'goal_pose_bridge.py')

    # -------------------------------------------------------------------------
    # Khai báo các đối số dòng lệnh (Launch Arguments) - Bằng Tiếng Việt
    # -------------------------------------------------------------------------
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Đường dẫn tuyệt đối hoặc tương đối tới file cấu hình bản đồ .yaml (Map YAML file)'
    )

    declare_planner_plugin_cmd = DeclareLaunchArgument(
        'planner_plugin',
        default_value='nav2_navfn_planner/NavfnPlanner',
        description='Tên plugin Global Planner cần sử dụng (ví dụ: nav2_navfn_planner/NavfnPlanner, nav2_smac_planner/SmacPlanner2D, custom_planner_example::CustomAStarPlanner)'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Đường dẫn đầy đủ tới file tham số cấu hình ROS 2 (nav2_params.yaml)'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Có sử dụng thời gian mô phỏng (clock /use_sim_time) hay không'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Tự động chuyển đổi trạng thái vòng đời (lifecycle) của nodes sang ACTIVE'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Có khởi chạy RViz2 để trực quan hóa đường đi hay không'
    )

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Đường dẫn file cấu hình giao diện hiển thị RViz (.rviz)'
    )

    declare_enable_rviz_click_cmd = DeclareLaunchArgument(
        'enable_rviz_click',
        default_value='True',
        description='Bật node cầu nối cho phép click chuột chọn 2D Goal Pose trong RViz2 để tìm đường đi A*'
    )

    # Khai báo tọa độ cố định cho điểm Start và Goal (Để trống để ưu tiên đọc từ file params_file / nav2_params.yaml)
    declare_start_x_cmd = DeclareLaunchArgument(
        'start_x',
        default_value='',
        description='Tọa độ X cố định của điểm xuất phát (Start X). Để trống để lấy từ config/nav2_params.yaml'
    )

    declare_start_y_cmd = DeclareLaunchArgument(
        'start_y',
        default_value='',
        description='Tọa độ Y cố định của điểm xuất phát (Start Y). Để trống để lấy từ config/nav2_params.yaml'
    )

    declare_goal_x_cmd = DeclareLaunchArgument(
        'goal_x',
        default_value='',
        description='Tọa độ X cố định của điểm đích đến (Goal X). Để trống để lấy từ config/nav2_params.yaml'
    )

    declare_goal_y_cmd = DeclareLaunchArgument(
        'goal_y',
        default_value='',
        description='Tọa độ Y cố định của điểm đích đến (Goal Y). Để trống để lấy từ config/nav2_params.yaml'
    )

    declare_auto_plan_cmd = DeclareLaunchArgument(
        'auto_plan',
        default_value='',
        description='Tự động tính toán đường đi giữa điểm Start và Goal cố định khi khởi động. Để trống để lấy từ config/nav2_params.yaml'
    )

    declare_marker_diameter_cmd = DeclareLaunchArgument(
        'marker_diameter',
        default_value='',
        description='Đường kính hình tròn đánh dấu điểm đầu (Xanh lá) và điểm cuối (Đỏ) trên RViz2 (mét). Để trống để lấy từ config/nav2_params.yaml'
    )

    declare_planner_id_cmd = DeclareLaunchArgument(
        'planner_id',
        default_value='',
        description='Tên plugin tìm đường cần chạy ("GridBased" cho A* hoặc "SCSO" cho SCSO). Để trống để lấy từ config/nav2_params.yaml'
    )

    # -------------------------------------------------------------------------
    # Các biến cấu hình từ Launch Arguments
    # -------------------------------------------------------------------------
    map_yaml_file = LaunchConfiguration('map')
    planner_plugin = LaunchConfiguration('planner_plugin')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config')
    enable_rviz_click = LaunchConfiguration('enable_rviz_click')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    auto_plan = LaunchConfiguration('auto_plan')
    marker_diameter = LaunchConfiguration('marker_diameter')
    planner_id = LaunchConfiguration('planner_id')

    # -------------------------------------------------------------------------
    # Định nghĩa các Node trong hệ thống
    # -------------------------------------------------------------------------

    # 0. Static Transform Publisher: Cung cấp TF map -> base_link
    # Giúp global_costmap khởi tạo thành công mà không cần Gazebo mô phỏng robot hay AMCL
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_base_link',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    # 1. Map Server: Đọc bản đồ tĩnh YAML/PGM và xuất bản ra topic /map
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml_file
            }
        ]
    )

    # 2. Planner Server: Tính toán đường đi toàn cục với thuật toán A*
    planner_server_node = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'GridBased.plugin': planner_plugin
            }
        ]
    )

    # 3. Lifecycle Manager: Quản lý vòng đời khởi động của map_server và planner_server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'planner_server']
        }]
    )

    # 4. RViz2: Giao diện trực quan hóa 2D/3D (Bản đồ, Costmap, A* Plan)
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 5. Goal Pose Bridge: Node tiện ích bắt sự kiện click 2D Goal Pose từ RViz2 và hỗ trợ điểm cố định
    def setup_goal_bridge(context):
        params_file_path = LaunchConfiguration('params_file').perform(context)
        enable_rviz_click_val = LaunchConfiguration('enable_rviz_click').perform(context).lower() in ('true', '1')

        if not enable_rviz_click_val:
            return []

        extra_params = {
            'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
        }

        # Kiểm tra nếu người dùng truyền đè tham số qua dòng lệnh CLI
        for param_name, cast_fn in [
            ('start_x', float),
            ('start_y', float),
            ('goal_x', float),
            ('goal_y', float),
            ('auto_plan', lambda v: v.lower() in ('true', '1')),
            ('marker_diameter', float),
            ('planner_id', str)
        ]:
            val_str = LaunchConfiguration(param_name).perform(context).strip()
            if val_str != '':
                extra_params[param_name] = cast_fn(val_str)

        node_params = [params_file_path, extra_params]

        if is_installed:
            node = Node(
                package='nav2_astar_demo',
                executable='goal_pose_bridge.py',
                name='goal_pose_bridge',
                output='screen',
                parameters=node_params
            )
        else:
            cmd = [
                sys.executable, goal_bridge_script,
                '--ros-args',
                '--params-file', params_file_path
            ]
            for k, v in extra_params.items():
                cmd.extend(['-p', f'{k}:={v}'])

            node = ExecuteProcess(
                cmd=cmd,
                output='screen'
            )

        return [node]

    # -------------------------------------------------------------------------
    # Tập hợp LaunchDescription
    # -------------------------------------------------------------------------
    ld = LaunchDescription()

    # Thêm các tham số cấu hình
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_planner_plugin_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_enable_rviz_click_cmd)
    ld.add_action(declare_start_x_cmd)
    ld.add_action(declare_start_y_cmd)
    ld.add_action(declare_goal_x_cmd)
    ld.add_action(declare_goal_y_cmd)
    ld.add_action(declare_auto_plan_cmd)
    ld.add_action(declare_marker_diameter_cmd)
    ld.add_action(declare_planner_id_cmd)

    # Thêm các nodes tối giản theo yêu cầu
    ld.add_action(static_tf_node)
    ld.add_action(map_server_node)
    ld.add_action(planner_server_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(rviz_node)
    ld.add_action(OpaqueFunction(function=setup_goal_bridge))

    return ld
