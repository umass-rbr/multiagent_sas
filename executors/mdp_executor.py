current_location = None

def location_handler(location):
    current_location = location

def excecute(policy):
    rospy.Subscriber("monitor/current_location", Location, location_handler)
    
    
