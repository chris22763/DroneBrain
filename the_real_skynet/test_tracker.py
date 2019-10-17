import math
sidereal_day_hours = 23.9344699
rads_per_sec = 2*math.pi/sidereal_day_hours/3600
# print(rads_per_sec)


import numpy as np
time = np.linspace(0,200*60,200*60)
length_in_cm = 20

def yt(length, ts):
    return length * np.tan(rads_per_sec * ts)

def ypt(length, ts):
    cm_per_sec = length/np.cos(rads_per_sec * ts)**2* rads_per_sec  # chain rule
    return cm_per_sec

import matplotlib.pyplot as plt
lengths_in_cm = [29,32,35]

# plot bolt length vs. time
# for length in lengths_in_cm:
#     y = yt(length, time)
#     plt.plot(time/60.0, y, label=str(length))
# plt.legend()
# plt.grid(color='0.7')
# plt.ylabel('Bolt insertion (cm)')
# plt.xlabel('Time since start (min)')
# plt.show()

# for length in lengths_in_cm:
#     y = ypt(length, time)
#     plt.plot(time/60.0, y*60.0, label='Length={}cm'.format(length))
# plt.legend()
# plt.grid(color='0.7')
# plt.ylabel('Bolt insertion rate (cm/min)')
# plt.xlabel('Time since start (min)')
# plt.show()

# print(rotations_per_cm)

# plt.rcParams.update({'font.size':16})
# for length in lengths_in_cm:
#     rotations_per_second = rotations_per_cm * ypt(length, time)
#     plt.plot(time/60.0, rotations_per_second*60,label='Length={}cm'.format(length))
#     plt.legend()
#     plt.grid(color='0.7')
#     plt.ylabel('Required rotation rate (rotations/min)')
#     plt.xlabel('Time since start (min)')
# plt.title('Rotation rate with 1/4-20 thread')
# plt.show()
# plt.savefig('rotation_rate.png')

# real calculation for my apparatus
# rotations_per_second = rotations_per_cm * ypt(29.113, time)
# seconds_per_step = 1.0/(rotations_per_second*4076)
rada = 2*math.pi / 23.9344699 / 3600
radb = 2*math.pi / 1436.06819 / 60
radc = 2*math.pi / 86164.0916

print(rada)
print(radb)
print(radc)
steps = 4076

rotations_per_cm = 1.25
cm_to_mm = 10
angle = 2*math.pi / 86164.0916
_sin = ((angle*time)/2)
a = (2 * 29) * np.sin(_sin)
delta_a = [(pt - a[i-1 if i > 0 else 0]) for i, pt in enumerate(a)]

rotations_per_second = [steps * dps for dps in delta_a]

sec_per_step = [(1/rps)*100000 if rps > 0 else 0 for rps in rotations_per_second]
pre_scaler = 256
sps_16mhz = [16000000/(sps*pre_scaler) if sps > 0 else 0  for sps in sec_per_step]
print(sum(delta_a))

graph = [sec_per_step]  # pheta_t, rotations_per_second,

for g in graph:

# print(rotations_per_second)
# print(4076*rotations_per_second)
    plt.plot(time/60.0, g)
plt.grid(color='0.7')
plt.ylabel('seconds per step')
plt.xlabel('Time since start (min)')
plt.show()

