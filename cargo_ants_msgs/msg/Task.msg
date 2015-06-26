uint64 task_id

# The vehicle which should visit the given locations.
#
string vehicle

# The locations to be visited, one after the other. There must be at
# least one location, in which case the vehicle should go from wherever
# it is now to that goal location.
#
Goal[] goals
