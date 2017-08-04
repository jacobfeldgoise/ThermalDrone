from dronekit import connect

# Connect to UDP endpoint (and wait for default attributes to accumulate)
print 'Connecting to Solo ...'
vehicle = connect('udpin:0.0.0.0:14550', wait_ready=True)
print vehicle
