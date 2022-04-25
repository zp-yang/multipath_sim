# %%
import os
import numpy as np
import json
import matplotlib.pyplot as plt

data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 


# %%
aff_pos_file = data_dir + "hk_data_affect_pos.json"

aff_pos = {}
with open(aff_pos_file, "r") as fs:
    aff_pos = json.load(fs)

grd_truth = {}
with open(data_dir+"car_ground_truth.json", "r") as fs:
    grd_truth = json.load(fs)

gps_log = {}
with open(data_dir+"hk_data_car_gps_log.json", "r") as fs:
    gps_log = json.load(fs)

marker_size = 20
plt.figure(figsize=(30,15))
plt.subplot(1,3,1)
plt.plot(grd_truth["lon"], grd_truth["lat"], "-")
plt.plot(grd_truth["lon"][0], grd_truth["lat"][0], "r.", markersize=marker_size)
plt.plot(grd_truth["lon"][-1], grd_truth["lat"][-1], "g.", markersize=marker_size)

# plt.subplot(1,3,2)
plt.plot(gps_log["lon"], gps_log["lat"], '.')
plt.plot(gps_log["lon"][0], gps_log["lat"][0], "m.", markersize=marker_size)
plt.plot(gps_log["lon"][-1], gps_log["lat"][-1], "c.", markersize=marker_size)

plt.subplot(1,3,2)
plt.plot(aff_pos["lon"], aff_pos["lat"], ".")
plt.plot(aff_pos["lon"][0], aff_pos["lat"][0], "r.", markersize=marker_size)
plt.plot(aff_pos["lon"][-1], aff_pos["lat"][-1], "g.", markersize=marker_size)

# %%
