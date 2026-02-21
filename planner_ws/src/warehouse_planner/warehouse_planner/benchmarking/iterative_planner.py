# Run with iterative planner


# Station parameters
# 0 = A
# 1 = B
# 2 = C
# 3 = D
# 4 = E
# 5 = F
# 6 = G



from training_simulator import Simulator


battery_threshold = 15.0

env = Simulator()
for i in range(env.num_packages):

    # Check for charging
    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    
    # Move to A
    env.cmd("MOVE_TO", 0) # A

    # Check for charging
    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
        env.cmd("MOVE_TO", 0) # A
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    
    # Pick at A
    while True:
        if env.packages[i].availability:
            env.cmd("PICK_A", i)
            break
        env.cmd("WAIT", -1) 

    # Check for charging
    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    
    # move to B
    env.cmd("MOVE_TO", 1) # B

    # Check for charging
    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
        env.cmd("MOVE_TO", 1) # B
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    
    # drop at B
    env.cmd("DROP", i)

    # Check for charging
    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
        env.cmd("MOVE_TO", 1) # B
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    
    # pick at B
    while True:
        env.cmd("WAIT", -1)
        if env.packages[i].availability:
            env.cmd("PICK", i)
            break

    # Check for charging
    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    
    # move to G if failedmodel
    if env.packages[i].next_location == "G":
        env.cmd("MOVE_TO", 6) # G

        # Check for charging
        if env.robot_battery < battery_threshold:
            env.cmd("MOVE_TO", 5) # F
            env.cmd("CHARGE", -1) 
            env.cmd("MOVE_TO", 6) # G
        elif env.robot_battery < 0.0:
            raise Exception("Battery is empty !!!")
        
        # Drop at G
        env.cmd("DROP", i)

        # Check for charging
        if env.robot_battery < battery_threshold:
            env.cmd("MOVE_TO", 5) # F
            env.cmd("CHARGE", -1) 
            env.cmd("MOVE_TO", 6) # G
        elif env.robot_battery < 0.0:
            raise Exception("Battery is empty !!!")
        
        # Pick at G
        while True:
            env.cmd("WAIT", -1)
            if env.packages[i].availability:
                env.cmd("PICK", i)
                break

        # Check for charging
        if env.robot_battery < battery_threshold:
            env.cmd("MOVE_TO", 5) # F
            env.cmd("CHARGE", -1) 
        elif env.robot_battery < 0.0:
            raise Exception("Battery is empty !!!")
    
    # move to C
    env.cmd("MOVE_TO", 2) # C

    # Check for charging
    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
        env.cmd("MOVE_TO", 2) # C
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    
    # drop at C
    env.cmd("DROP", i)

    # Check for charging
    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
        env.cmd("MOVE_TO", 2) # C
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    
    # pick at C
    while True:
        env.cmd("WAIT", -1)
        if env.packages[i].availability:
            env.cmd("PICK", i)
            break

    # Check for charging
    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    
    # move to D or E
    if env.packages[i].shipping_type == "standard":
        env.cmd("MOVE_TO", 3) # D

        # check for charging
        if env.robot_battery < battery_threshold:
            env.cmd("MOVE_TO", 5) # F
            env.cmd("CHARGE", -1) 
            env.cmd("MOVE_TO", 3) # D
        elif env.robot_battery < 0.0:
            raise Exception("Battery is empty !!!")
        
        # drop at D
        env.cmd("DROP", i)

        # check for charging
        if env.robot_battery < battery_threshold:
            env.cmd("MOVE_TO", 5) # F
            env.cmd("CHARGE", -1) 
            env.cmd("MOVE_TO", 3) # D
        elif env.robot_battery < 0.0:
            raise Exception("Battery is empty !!!")
        
        # wait if success at D
        while True:
            env.cmd("WAIT", -1)
            if env.packages[i].lifecycle_state == "FINISHED":
                break
            elif env.packages[i].lifecycle_state == "FAILED":
                env.cmd("PICK", i)

                # check for charging
                if env.robot_battery < battery_threshold:
                    env.cmd("MOVE_TO", 5) # F
                    env.cmd("CHARGE", -1) 
                elif env.robot_battery < 0.0:
                    raise Exception("Battery is empty !!!")
                
                # move to E
                env.cmd("MOVE_TO", 4) # E

                # check for charging
                if env.robot_battery < battery_threshold:
                    env.cmd("MOVE_TO", 5) # F
                    env.cmd("CHARGE", -1) 
                    env.cmd("MOVE_TO", 4) # E
                elif env.robot_battery < 0.0:
                    raise Exception("Battery is empty !!!")
                
                # drop at E
                env.cmd("DROP", i)

                break

    else:
        # move to E
        env.cmd("MOVE_TO", 4) # E

        # check for charging
        if env.robot_battery < battery_threshold:
            env.cmd("MOVE_TO", 5) # F
            env.cmd("CHARGE", -1) 
            env.cmd("MOVE_TO", 4) # E
        elif env.robot_battery < 0.0:
            raise Exception("Battery is empty !!!")
                
        # drop at E
        env.cmd("DROP", i)
 


while True:
    env.cmd("WAIT", -1)
    if env.packages[19].next_location == "FINISH":
        print(f"Best case time in iterative processing: {env.time}")
        break