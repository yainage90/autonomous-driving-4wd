"""Time overrides shared by integrated and standalone navigation launches."""
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def sim_time():
    return ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)


def timed_params(source, *nodes):
    time = LaunchConfiguration('use_sim_time')
    rewrites = {'use_sim_time': time}
    # Explicit paths also cover custom YAML files without an existing time key.
    for node in nodes:
        rewrites[f'{node}.ros__parameters.use_sim_time'] = time
    return ParameterFile(RewrittenYaml(
        source_file=source, param_rewrites=rewrites, convert_types=True,
    ), allow_substs=False)
