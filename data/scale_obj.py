#%%
import pandas as pd


scale = 1000

df = pd.read_csv("Code/Lidar_simulator_py/data/obj_scale_orgn", sep=" ", names =["v", "x", "y", "z"], )
df[["x", "y", "z"]] = df[["x", "y", "z"]] * scale
df.to_csv(f"Code/Lidar_simulator_py/data/obj_scale_{scale}",sep= " ", index =False, header = False)