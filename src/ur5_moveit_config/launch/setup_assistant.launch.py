from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


# Start the MoveIt Setup Assistant.
def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "group4_ur5",
            package_name="ur5_moveit_config",
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )
    return generate_setup_assistant_launch(moveit_config)
