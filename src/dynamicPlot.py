import serial
import os
import sys
from matplotlib import pyplot as plt
import pandas as pd

ser = serial.Serial('COM8')  # open serial port 

angleCols = [5, 6]
constantCols = [1, 2, 3, 4]
xcol = 0

dataframes:list = []

poster, axs = plt.subplots(2,6,sharex=True,sharey=True)
plt.legend()
plt.tight_layout()
plt.ion()

i = 0

while True:
    if dataframes.append(pd.DataFrame(ser.read_all())):
        for df in dataframes:
            print("Plotting x =", xcol, "y =", constantCols)
            axs[0,i] = df.plot(x=xcol, y=angleCols, grid=True)
            axs[1,i] = df.plot(x=xcol, y=constantCols, grid=True)

            axs[0,i].set_xlabel("time (s)")
            axs[0,i].set_ylabel("angle")
            axs[1,i].set_xlabel("time (s)")
            axs[1,i].set_ylabel("controller output")
            if i < 6:
                i += 1
            else:
                i = 0

            