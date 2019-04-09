import numpy as np
import matplotlib.pyplot as plt
import math


Fs = 1000
f = 5
sample = 1000
x = np.arange(sample)
y = np.sin(2 * np.pi * f * x / Fs)
y_noisy = y + np.random.normal(0,0.1,1000)
# plt.plot(x, y_noisy)
# plt.plot(x, y, 'r', linewidth=2)
# plt.xlabel('sample(n)')
# plt.ylabel('y')
# plt.show()

# f = open("observation.txt", "w")
#
# for i in range(sample):
#     f.write(str(y_noisy[i]) + "\n")
# f.close()
#
# f = open("original_data.txt", "w")
# for i in range(sample):
#     f.write(str(y[i]) + "\n")
# f.close()


with open("FilterOutput.txt") as f:
    content = f.readlines()
content = [x.strip() for x in content]
plt.plot(y_noisy, 'r')
plt.plot(content)
plt.show()