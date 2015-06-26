uint64 task_request_id

# One or more vehicles, one of which should visit the given locations
# in sequence. If there is more than one vehicle, the task scheduler
# will decide which one to use. If there is no vehicle, the request
# will be ignored.
#
string[] vehicles

# One or more locations, to be visited in sequence. Typically, you may
# want to specify a pick-up and a drop-off location. You can also give
# a single location (which is like a simple go-to command), or a whole
# sequence (if you need to follow a certain order of tasks). If there is no
# location, the request will be ignored.
#
string[] locations
