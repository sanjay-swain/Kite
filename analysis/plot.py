import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("output/run.csv")
for body, g in df.groupby("body"):
    plt.plot(g["t"], g["pz"], label=f"body {body}")
plt.xlabel("t [s]"); plt.ylabel("z [m]"); plt.legend()

plt.savefig("output/plot.png", dpi=150)