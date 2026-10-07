# "흐름 1: 지도 작성" launch 파일
# 구성: robot_state_publisher + twist_mux + odom_tu_lenh + SLAM Toolbox (online async)
# 선택: 라이다(bat_lidar), 모터+원격 조종(bat_dong_co), 카메라(bat_camera)
#   bat_dong_co 는 dieu_khien 과 teleop_web 을 함께 켜고 끈다. dieu_khien 이 꺼진 상태에서
#   teleop_web 만 켜지면 로봇은 서 있는데 odom_tu_lenh 가 /cmd_vel 을 적분해 지도가 틀어지기 때문이다.
# 필수: 원격 조종 웹의 접근 코드 access_code (기본값 없음, 코드는 파일에 쓰지 않는다;
#       teleop_web 이 꺼져 있어도 항상 검사한다)
# Nav2, cam_bien, logic_quyet_dinh 는 이 launch 에 포함하지 않는다.
#
# 사용 예:
#   ros2 launch my_bot mapping_my_bot.launch.py access_code:=<코드> bat_dong_co:=true

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def start_teleop_web(context, *args, **kwargs):
    """access_code 를 검사한 뒤 teleop_web 노드를 만든다.

    access_code 가 없거나 비어 있으면 teleop_web 은 인증 없이 동작하므로,
    그런 상태로 로봇이 움직이지 않도록 launch 전체를 오류로 중단한다.
    """
    access_code = context.launch_configurations.get('access_code')
    if access_code is None or not access_code.strip():
        raise RuntimeError(
            "access_code 가 필요합니다. 예: ros2 launch my_bot mapping_my_bot.launch.py "
            "access_code:=<코드>  (비어 있는 값은 허용하지 않습니다: 인증 없이 로봇이 "
            "움직이는 것을 막기 위함)")
    return [
        Node(
            package='my_bot',
            executable='teleop_web.py',
            name='teleop_web',
            output='screen',
            # dieu_khien 과 같은 조건(bat_dong_co)으로 켜고 끈다 (둘이 항상 같이 움직이도록)
            condition=IfCondition(LaunchConfiguration('bat_dong_co')),
            # 숫자만으로 된 코드가 정수로 해석되지 않도록 문자열(str)로 강제한다
            parameters=[{'access_code': ParameterValue(access_code, value_type=str)}]),
    ]


def generate_launch_description():
    # my_bot 패키지의 share 디렉터리 가져오기
    bringup_dir = get_package_share_directory('my_bot')

    slam_params_file = LaunchConfiguration('slam_params_file')
    bat_lidar = LaunchConfiguration('bat_lidar')
    bat_dong_co = LaunchConfiguration('bat_dong_co')
    bat_camera = LaunchConfiguration('bat_camera')

    declare_access_code_cmd = DeclareLaunchArgument(
        'access_code',
        description='Access code for teleop_web (required, no default; empty is rejected)')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml'),
        description='Full path to the ROS2 parameters file to use for slam_toolbox')

    declare_bat_lidar_cmd = DeclareLaunchArgument(
        'bat_lidar', default_value='true',
        description='Launch the YDLidar driver (ydlidar_ros2_driver)')

    declare_bat_dong_co_cmd = DeclareLaunchArgument(
        'bat_dong_co', default_value='false',
        description='Launch dieu_khien (motor control, needs Raspberry Pi GPIO) AND teleop_web together')

    # max_wheel_speed: PWM 듀티 1.0 일 때의 바퀴 선속도 [m/s]. dieu_khien 에 문자열이 아니라 실수로 전달한다.
    # (dieu_khien.py 자체의 기본값 0.23 은 바꾸지 않고, 이 launch 에서만 0.437 로 덮어쓴다)
    declare_max_wheel_speed_cmd = DeclareLaunchArgument(
        'max_wheel_speed', default_value='0.437',
        description='Wheel linear speed [m/s] at PWM duty 1.0, passed to dieu_khien')

    declare_bat_camera_cmd = DeclareLaunchArgument(
        'bat_camera', default_value='false',
        description='Launch camera_csi (CSI camera node)')

    # 로봇 모델 (robot_state_publisher + joint_state_publisher). 지도 작성은 항상 실제 시계 사용
    rsp_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items())

    # 속도 명령 우선순위 중재 (출력: /cmd_vel)
    twist_mux_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'twist_mux.launch.py')))

    # 엔코더가 없으므로 /cmd_vel 을 적분해 /odom 과 TF(odom -> base_footprint) 를 만든다
    odom_cmd = Node(
        package='my_bot',
        executable='odom_tu_lenh.py',
        name='odom_tu_lenh',
        output='screen')

    # SLAM Toolbox online async. 샘플 launch 의 use_sim_time 기본값은 'true' 이므로 반드시 'false' 로 덮어쓴다
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': 'false',
                          'slam_params_file': slam_params_file}.items())

    # 라이다 드라이버: 이 PC 에는 패키지가 없을 수 있으므로 import 시점이 아니라
    # 실행 시점에 FindPackageShare 로 경로를 찾는다 (bat_lidar:=true 일 때만 평가)
    lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ydlidar_ros2_driver'),
                                  'launch', 'ydlidar_launch.py'])),
        condition=IfCondition(bat_lidar))

    dong_co_cmd = Node(
        package='my_bot',
        executable='dieu_khien.py',
        name='dieu_khien',
        output='screen',
        # access_code 와 같은 방식으로 타입을 명시한다 (float 로 강제)
        parameters=[{'max_wheel_speed': ParameterValue(LaunchConfiguration('max_wheel_speed'), value_type=float)}],
        condition=IfCondition(bat_dong_co))

    camera_cmd = Node(
        package='my_bot',
        executable='camera_csi.py',
        name='camera_csi',
        output='screen',
        condition=IfCondition(bat_camera))

    # LaunchDescription 을 생성하고 액션을 채운다
    ld = LaunchDescription()

    # launch 옵션 선언
    ld.add_action(declare_access_code_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_bat_lidar_cmd)
    ld.add_action(declare_bat_dong_co_cmd)
    ld.add_action(declare_max_wheel_speed_cmd)
    ld.add_action(declare_bat_camera_cmd)

    # access_code 검사를 가장 먼저 둔다 (없으면 다른 노드가 하나도 뜨기 전에 중단)
    ld.add_action(OpaqueFunction(function=start_teleop_web))

    # 항상 실행되는 노드
    ld.add_action(rsp_cmd)
    ld.add_action(twist_mux_cmd)
    ld.add_action(odom_cmd)
    ld.add_action(slam_cmd)

    # 조건부 노드
    ld.add_action(lidar_cmd)
    ld.add_action(dong_co_cmd)
    ld.add_action(camera_cmd)

    return ld
