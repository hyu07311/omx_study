"""launch 파일들이 같이 쓰는 MoveIt 설정 (move_group / RViz 가 같은 설정을 써야 함)"""
from moveit_configs_utils import MoveItConfigsBuilder

# launch 인자 kinematics:= 값 → kinematics 설정 파일
KINEMATICS_FILES = {
    'kdl': 'config/kinematics.yaml',
    'lma': 'config/kinematics_lma.yaml',
    'trac_ik': 'config/kinematics_trac_ik.yaml',
}


def moveit_config(kinematics='kdl'):
    return (MoveItConfigsBuilder('omy', package_name='omy_moveit_config')
            # .setup_assistant: omy_description/urdf/omy.urdf.xacro use_sim:=false variant:=f3m
            .robot_description()
            .robot_description_semantic(file_path='config/omy.srdf')
            .robot_description_kinematics(file_path=KINEMATICS_FILES[kinematics])
            .joint_limits(file_path='config/joint_limits.yaml')
            .trajectory_execution(file_path='config/moveit_controllers.yaml')
            .planning_pipelines(pipelines=['ompl'])
            .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
            .to_moveit_configs())
