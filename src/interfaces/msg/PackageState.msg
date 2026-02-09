int32 package_idx        # 0..19
string shipping_type     # "standard"|"express"
string current_location  # "ROBOT","A"...,"FINISH"
string next_location     # "A"...,"FINISH"
string lifecycle_state   # "SPAWNED","WAITING","PROCESSING","READY","FINISHED","FAILED"
bool availability        # ready for pickup (station output)
