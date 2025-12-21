from stable_baselines3 import PPO
from env import EcoDrivingEnv
import pandas as pd
import numpy as np

def train_and_export():
    env = EcoDrivingEnv()
    
    # Increase timesteps to ensure it learns to drive fast
    # High entropy (0.05) ensures it tries full throttle initially
    model = PPO(
        "MlpPolicy", 
        env, 
        verbose=1, 
        learning_rate=5e-5,
        ent_coef=0.05, 
        batch_size=128
    )
    
    print("Training Model (100k steps)...")
    # Training for longer is safer to ensure it finishes
    model.learn(total_timesteps=100000)
    print("Training Finished.")
    
    print("\nRunning Optimized Validation Lap...")
    obs, _ = env.reset()
    done = False
    
    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        
        # Optional: Print progress during validation
        # print(f"Speed: {obs[0]*3.6:.1f} km/h, Gear: {obs[4]}")

    # Export
    log_data = env.unwrapped.history 
    
    if len(log_data) > 0:
        df_res = pd.DataFrame(log_data)
        output_file = 'optimized_run.csv'
        df_res.to_csv(output_file, index=False)
        
        final_speed = df_res['speed_kmh'].iloc[-1]
        total_time = df_res['time'].iloc[-1]
        max_speed = df_res['speed_kmh'].max()
        
        print(f"\nOptimization complete.")
        print(f"Total Time: {total_time}s")
        print(f"Max Speed: {max_speed:.2f} km/h")
        print(f"Final Speed: {final_speed:.2f} km/h")
        print(f"Data saved to {output_file}")
    else:
        print("No data recorded.")

if __name__ == "__main__":
    train_and_export()