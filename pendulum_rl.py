import gymnasium as gym
from stable_baselines3 import PPO

# Create the environment
env = gym.make("CartPole-v1", render_mode="human")

print("Environment created successfully!")

# Create the The PPO Agent
model = PPO("MlpPolicy", env, verbose=1)

print("Brain initialized and ready to learn!")
print("Training started... watch the terminal!")
model.learn(total_timesteps=10000)

# Save the brain
model.save("ppo_cartpole_model")

print("Training finished and model saved!")

# 1. Reset the world to start the test
obs, info = env.reset()

# 2. Run the simulation for 1,000 steps
for i in range(1000):
  
    action, _states = model.predict(obs, deterministic=True)
    
    obs, reward, terminated, truncated, info = env.step(action)
    
    env.render()
    
    # If the pole falls, reset it to try again
    if terminated or truncated:
        obs, info = env.reset()


env.close()
