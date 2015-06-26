# The idea is for the vehicle_control node to track the given
# trajectory almost blindly, but using emergency stops if necessary.
# The trajectory is computed by a node which can adapt the path to
# changes in the environment, e.g. via elastic planning.
# ------------------------------------------------------------

# Trajectory points encode a time, pose, and velocity.  This specifies
# when the vehicle should be at which state and at which velocity.
# Not quite sure yet whether this should be in local or site
# coordinates.
#
#
# Sending an empty list of points will make the vehicle stop wherever
# it is.
#
TrajectoryPoint[] points
