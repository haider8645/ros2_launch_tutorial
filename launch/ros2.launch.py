import yaml

from launch import LaunchDescription
from launch_ros.actions import Node  
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import OpaqueFunction, DeclareLaunchArgument 
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition  

#Helper function to read a yaml config file
def load_yaml_file(yaml_file_path):
    try:
        with open(yaml_file_path, 'r') as file:
            return yaml.load(file)
    except EnvironmentError as e: # parent of IOError, OSError *and* WindowsError where available
        print(str(e))
        return None    

def launch_setup(context, *args, **kwargs):

  #Example: Read the config yaml file and do something based on the values       
  joy_config = PathJoinSubstitution(
      [FindPackageShare('ros2_launch_tutorial'), "config", "joy-params.yaml"]
  ) 
  #Read the yaml file's ros parameters
  joy_params = load_yaml_file(str(joy_config.perform(context)))['joy']['ros__parameters'] 
  #You an build login based on the config file values to adapt your launch file behavior  
  print("Joy Params: ")
  print(joy_params)  
  
  #Example: Get the launch file arguments declared in the generate_launch_description() method
  description_package = LaunchConfiguration('description_package')
  description_file = LaunchConfiguration('description_file')
  prefix = LaunchConfiguration('prefix') 
 
  #Example: Access the value of a launch argument from the LaunchContext  
  print("description_package:")
  print(str(LaunchConfiguration('description_package').perform(context)))
  
  #Example: Use conditionals
  #Usage: ros2 launch ros2_launch_tutorial ros2.launch.py conditional_demo:=false
  if(IfCondition(LaunchConfiguration('conditional_demo')).evaluate(context)):
    print("The conditional_demo value was set to True")  
  else:
    print("The conditional_demo value was set to False")  
      
  #Example: Get URDF via xacro
  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name="xacro")]),
          " ",
          PathJoinSubstitution(
              [FindPackageShare(description_package), "urdf", description_file]
          ),
          " ",
          "prefix:=",
          prefix,
      ]
  )
  
  rviz_config = PathJoinSubstitution(
        [FindPackageShare('ros2_launch_tutorial'), "rviz", "rrbot.rviz"]
    )
 
  robot_state_publisher_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='both',
      parameters=[{'robot_description': robot_description_content}]
      )
  
  joint_state_publisher_gui_node= Node(
      package='joint_state_publisher_gui',
      executable='joint_state_publisher_gui',
      name='joint_state_publisher_gui',
      output='both'
      )
  
  rviz_node = Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      output="log",
      arguments=["-d", rviz_config] # You can directly use the PathJoinSubstitution object 
  )
  
  joy_node = Node(
    package='joy',
    executable='joy_node',
    name='joy',
    output='both',
    parameters=[joy_config] # You can directly use the PathJoinSubstitution object 
  )
  
  #Setup your launch file description nodes
  return [robot_state_publisher_node, joint_state_publisher_gui_node, rviz_node, joy_node]  
  
# Main function serving as a starting point for the launch file 
def generate_launch_description(): 
       
  return LaunchDescription([
      
    #Declare all arguments here. The arguments can be accessed using LaunchConfiguration() in the launch_setup() method  
    DeclareLaunchArgument(
        "description_package",
        default_value="ros2_launch_tutorial",
        description="Description package with robot URDF/xacro files. Usually the argument \
    is not set, it enables use of a custom description."
    ),
        
    DeclareLaunchArgument(
        "description_file",
        default_value="rrbot.urdf.xacro",
        description="URDF/XACRO description file with the robot."
    ),   

    DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated."
    ),
    
    DeclareLaunchArgument(
        "conditional_demo",
        default_value="true",
        description="A demo argument to show an example of the IfCondition module."
    ),

    OpaqueFunction(function = launch_setup)
    
    ])
  

