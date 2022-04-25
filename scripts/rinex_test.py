# %%
import numpy as np
import matplotlib.pyplot as plt
import georinex as gr

import os
data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 

obs_file = data_dir + "hk_data/20210517.light-urban.tste.ublox.m8t.GEJ.obs"

obs = gr.load(obs_file)
# %%
import json
gps_log = {}
with open(data_dir+"hk_data_car_gps_log.json", "r") as fs:
    gps_log = json.load(fs)

grd_truth = {}
with open(data_dir+"car_ground_truth.json", "r") as fs:
    grd_truth = json.load(fs)
# obs.to_netcdf(obs_file+".processed.nc", group="OBS")
# %%
plt.figure(figsize=(20,10))
plt.plot(obs.C1C[250:350, :].data, '.')
plt.legend(obs.sv[:].data)
plt.xlabel("time[s]")
plt.ylabel("distance[m]")
# %%
sv_range = range(12,13)
diff_pr = np.diff(obs.C1C.data, axis=0)
plt.figure(figsize=(20,20))
plt.subplot(3,1,1)
plt.plot(np.sqrt(diff_pr[:, sv_range]**2))
plt.legend(obs.sv[sv_range].data)


plt.subplot(3,1,2)
diff_cp = np.diff(obs.S1C.data, axis=0)
plt.plot(diff_cp[:, sv_range])
plt.legend(obs.sv[sv_range].data)

# plt.subplot(3,1,3)
# # diff_cp = np.diff(obs.S1C.data, axis=0)
# plt.plot(obs.S1C[:, sv_range])
# plt.legend(obs.sv[sv_range].data)
ax = plt.subplot(3,1,3)
ax.plot(gps_log["lon"], ".")
ax.plot(grd_truth["lon"])
x = len(gps_log["lon"])
ax.fill_between(range(x), 0, 1, where=np.isnan(obs.C1C.data[:, sv_range]))
# %%
