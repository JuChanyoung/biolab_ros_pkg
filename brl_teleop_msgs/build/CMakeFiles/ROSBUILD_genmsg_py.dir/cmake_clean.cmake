FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/brl_teleop_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/brl_teleop_msgs/msg/__init__.py"
  "../src/brl_teleop_msgs/msg/_Energy.py"
  "../src/brl_teleop_msgs/msg/_Pos.py"
  "../src/brl_teleop_msgs/msg/_Package.py"
  "../src/brl_teleop_msgs/msg/_Force.py"
  "../src/brl_teleop_msgs/msg/_Vel.py"
  "../src/brl_teleop_msgs/msg/_Common.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
