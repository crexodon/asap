from __future__ import annotations

import time

from env_benchmark import WarehouseMDPEnv
from masking_train import compute_action_mask, flat_to_type_param, station_param_to_station
from training_simulator import Simulator 



def _action_to_cmd(flat_action: int) -> str:
    atype, param = flat_to_type_param(int(flat_action))
    if atype == 0:
        return "WAIT"
    if atype == 1:
        return "CHARGE"
    if atype == 2:
        return f"MOVE_TO {station_param_to_station(param)}"
    if atype == 3:
        return f"PICK {param}"
    if atype == 4:
        return f"DROP {param}"
    if atype == 5:
        return f"PICK_A {param}"
    return "WAIT"


def main():
    ros = Simulator()

    model_name = "model.zip"
    model_path = f"../../models/{model_name}"

    decision_sleep_s = 0.0

    try:
        from sb3_contrib import MaskablePPO
    except Exception:

        raise

    env = WarehouseMDPEnv(ros)

    print(f"Loading model: {model_path}")
    model = MaskablePPO.load(str(model_path), env=env)

    # Initial obs
    obs, _ = env.reset()

    total_reward = 0.0
    total_steps = 0
    start_wall_time = time.monotonic()

    while True:
        # Build action mask
        mask = compute_action_mask(obs)

        # Predict with mask
        action, _ = model.predict(obs, action_masks=mask, deterministic=True)

        cmd = _action_to_cmd(int(action))
        print(f"Chosen action={int(action)} cmd='{cmd}'")

        # Execute a single env step (will block until done / wait interrupt)
        obs, reward, terminated, truncated, info = env.step(int(action))


        total_reward += reward
        total_steps += 1
        dt = info.get('dt', 0.0)  

        print(
            f"Step finished: reward={reward:.3f} dt={info.get('dt', 0.0):.3f} terminated={terminated} truncated={truncated}"
        )

        if terminated or truncated:
            end_wall_time = time.monotonic()
            wall_time_duration = end_wall_time - start_wall_time
            sim_time_duration = ros.episode_elapsed_s()
            
            status = "SUCCESSFUL" if terminated and not info.get('battery_depleted') else "CANCELED"
            
            print("\n" + "="*40)
            print(f"      EPISODE-SUMMARY")
            print("="*40)
            print(f"Status:           {status}")
            print(f"Gesamt-Reward:    {total_reward:.2f}")
            print(f"Schritte:         {total_steps}")
            print(f"Simulationszeit:  {sim_time_duration:.2f} s")
            print(f"Echtzeit:         {wall_time_duration:.2f} s")
            print("="*40 + "\n")
            
            print(f"Episode finished. Reward: {total_reward:.2f}, Time: {sim_time_duration:.2f}s")
            break

        time.sleep(decision_sleep_s)


if __name__ == "__main__":
    main()
