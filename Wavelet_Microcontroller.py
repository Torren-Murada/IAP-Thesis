# -*- coding: utf-8 -*-
"""
Created on Fri May 25 09:26:56 2025

@author: User
"""

import re
import matplotlib.pyplot as plt
import pandas as pd


plt.close("all")

file_path = "wavelet 0.08 step 3 decimal raw" 
file_path = "wavelet 0.2 step" 
file_path = "wavelet 0.08 step" 
with open(file_path, 'r') as f:
    lines = f.readlines()

# Extract frequency1 and frequency2 lines
frequency1_lines = [line for line in lines if 'frequency1' in line]
frequency2_lines = [line for line in lines if 'frequency2' in line]

def extract_frequencies(lines, label):
    return [float(re.search(rf'{label}:\s*(-?\d+\.\d+)', line).group(1)) for line in lines]

# Get values
frequency1_values = extract_frequencies(frequency1_lines, 'frequency1')
frequency2_values = extract_frequencies(frequency2_lines, 'frequency2')

df = pd.DataFrame({
    'frequency1': frequency1_values,
    'frequency2': frequency2_values
})

# Plot
plt.figure(figsize=(10, 5))
plt.plot(df['frequency1'], label='frequency1', linewidth=2)
plt.plot(df['frequency2'], label='frequency2', linewidth=2)
plt.xlabel('Sample Index')
plt.ylabel('Frequency (Hz)')
plt.title('Extracted Frequencies Wavelet 0.08 Step 2 Decimal Raw')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
