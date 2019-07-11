def reward_function(params):
    def good_speed_reward(direction_diff, speed):
        high = 5
        low = 2
        if direction_diff < 5:
            bspeed = high
        elif direction_diff < 30:
            bspeed = (high-low) * ((30 - direction_diff) / (30 - 5)) + low
        else:
            bspeed = low
            
        speed_diff = abs(speed - bspeed)
        if speed_diff > 5:
            return 0.1
        else:
            return (5 - speed_diff) * (5 - speed_diff) * (5 - speed_diff) / 125 + 0.1


    ###############################################################################
    '''
    Example of using waypoints and heading to make the car in the right direction
    '''
    
    import math
    
    # Read input variables
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    speed = params['speed']
    
    # Initialize the reward with typical value 
    reward = 1.0
    
    # Calculate the direction of the center line based on the closest waypoints
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]
    
    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]) 
    # Convert to degree
    track_direction = math.degrees(track_direction)
    
    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - heading)
    
    # Penalize the reward if the difference is too large
    if direction_diff > 20:
        reward *= 0.2
    if direction_diff > 10:
        reward *= 0.5
    
    if speed < 2:
        reward *= 0.05
    else:
        reward += (reward*good_speed_reward(direction_diff, speed))
    
    return float(reward)
    