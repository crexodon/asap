# calculate the time for an iterative planner. 

## --- Best case scenario --- ##
# Assumptions:
# - All proceesses are always successful
# - Always the minimum time was reached at all processes
# - robot goes charging when battery capacity is lower than 15%
# - Wait at stations until package becomes available

# Station parameters
# 0 = A
# 1 = B
# 2 = C
# 3 = D
# 4 = E
# 5 = F
# 6 = G

from training_simulator_bestcase import Simulator

battery_threshold = 15.0

env = Simulator()
for i in range(env.num_packages):
    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    env.cmd("MOVE_TO", 0) # A

    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
        env.cmd("MOVE_TO", 0) # A
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    while True:
        if env.packages[i].availability:
            env.cmd("PICK_A", i)
            break
        env.cmd("WAIT", -1) 

    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    env.cmd("MOVE_TO", 1) # B

    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    env.cmd("DROP", i)

    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    # wait until process is finished
    while True:
        env.cmd("WAIT", -1)
        if env.packages[i].availability:
            env.cmd("PICK", i)
            break

    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    env.cmd("MOVE_TO", 2) # C

    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    env.cmd("DROP", i)

    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    while True:
        env.cmd("WAIT", -1)
        if env.packages[i].availability:
            env.cmd("PICK", i)
            break

    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    if env.packages[i].shipping_type == "standard":
        env.cmd("MOVE_TO", 3) # D
    else:
        env.cmd("MOVE_TO", 4) # E

    if env.robot_battery < battery_threshold:
        env.cmd("MOVE_TO", 5) # F
        env.cmd("CHARGE", -1) 
    elif env.robot_battery < 0.0:
        raise Exception("Battery is empty !!!")
    env.cmd("DROP", i)

while True:
    env.cmd("WAIT", -1)
    if env.packages[19].next_location == "FINISH":
        print(f"Best case time in iterative processing: {env.time}")
        break