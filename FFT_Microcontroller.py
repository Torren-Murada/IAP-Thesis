# -*- coding: utf-8 -*-
"""
Created on Sun Jun  1 13:04:10 2025

@author: User
"""
import pandas as pd
import matplotlib.pyplot as plt
import re


plt.close("all")

file_path = "cable1_data.csv"
df = pd.read_csv(file_path, usecols=[2, 4], low_memory=False)

labels = df.iloc[:, 0].astype(str).str.strip().str.lower()
values = pd.to_numeric(df.iloc[:, 1], errors='coerce')

filtered_values = values[labels == 'frequency'].reset_index(drop=True)

# Split into frequency1 and frequency2
frequency1_values = filtered_values.iloc[::2].reset_index(drop=True)
frequency2_values = filtered_values.iloc[1::2].reset_index(drop=True)

min_len = min(len(frequency1_values), len(frequency2_values))
frequency1_values = frequency1_values[:min_len]
frequency2_values = frequency2_values[:min_len]

print("\n🔍 First 10 frequency1 values:", frequency1_values.head(10).tolist())
print("🔍 First 10 frequency2 values:", frequency2_values.head(10).tolist())

# Plot
if min_len == 0:
    print("No frequency pairs found to plot.")
else:
    plt.figure(figsize=(10, 5))
    plt.plot(frequency1_values, label='frequency1', linewidth=2)
    plt.plot(frequency2_values, label='frequency2', linewidth=2)
    plt.xlabel("Sample Index")
    plt.ylabel("Frequency (Hz)")
    plt.title("Frequencies Extracted From FFT")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()