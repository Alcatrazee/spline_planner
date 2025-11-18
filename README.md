# spline_planner
ros2 nav2 planner using spline_planner

# nav2 parameters
plugin: "spline_planner/SplinePlanner"
interpolation_resolution: 0.01  # distance between waypoints
spline_type: 2  # 0: bezier planner  1: cubic spline planner 2: quintic spline planner
planner_param: 0.4  # if bezier planner,not used; if cubic spline planner, it is control ratio; if quintic spline, it is final straight line length.