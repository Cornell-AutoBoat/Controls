# Boat position variables. Updated by the Serial Communication node with data from the GPS and compass.
tx, ty = 0.0, 0.0
heading = 0.0

# Publisher for Controls messages
cPub = None
# Publisher for Done messages
dPub = None

# Current motor signals being sent
sL, sR = 1500, 1500

# Stopped (Stop command sent so quit execution)
stopped = False

# Stop main loop
finish = False

# Controls which motion
motion = None

# Execution path
path = []

# Callback variable
callback = False