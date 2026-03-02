import os
import sys
from matplotlib import pyplot as plt
import pandas as pd

path = input("Enter the path to the CSV file: ").strip().strip('"').strip("'")

if not os.path.isfile(path):
    print("File not found:", path)
    sys.exit(1)

df = pd.read_csv(path, sep=',')

if df is None:
    print("Failed to read CSV. Try opening it in a text editor to check delimiter/encoding.")
    sys.exit(1)

print("Loaded file:", path)
print("Columns:", list(df.columns))
print(df.head(5))

def angleChange(x):
    out = []
    for i in x:
        i = i-360 if i > 180 else i
        out.append(i)
    return out

df2 = pd.DataFrame(df.apply(angleChange, axis=1,result_type="broadcast"))
print(df2.head(5))

# Plotting
angleCols = [5, 6]
constantCols = [1, 2, 3, 4]
xcol = 0

print("Plotting x =", xcol, "y =", constantCols)
fig, (ax1, ax2) = plt.subplots(2, sharex=True)
ax1 = df2.plot(x=xcol, y=angleCols, ax=ax1, grid=True)
ax2 = df2.plot(x=xcol, y=constantCols, ax=ax2, grid=True)

ax1.set_xlabel("time (s)")
ax1.set_ylabel("angle")
ax2.set_xlabel("time (s)")
ax2.set_ylabel("controller output")
plt.legend()
plt.tight_layout()
plt.show()