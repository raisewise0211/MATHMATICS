# Let's retry generating the plot of tanh(a)
import numpy as np
import matplotlib.pyplot as plt

# Define the tanh function
def tanh(a):
    return np.tanh(a)

# Generate values for a
a_values = np.linspace(-5, 5, 400)

# Calculate corresponding tanh values
tanh_values = tanh(a_values)

# Plot the tanh function
plt.figure(figsize=(8, 6))
plt.plot(a_values, tanh_values, label="tanh(a)", color='blue')
plt.title("Graph of tanh(a)")
plt.xlabel("a")
plt.ylabel("tanh(a)")
plt.grid(True)
plt.axhline(0, color='black',linewidth=0.5)
plt.axvline(0, color='black',linewidth=0.5)
plt.legend()
plt.show()
