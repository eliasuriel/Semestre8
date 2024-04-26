import numpy as np
import matplotlib.pyplot as plt

def w(t, sigma):
    return np.random.normal(loc=0, scale=sigma)

def monte_carlo_derivative(sigma, num_samples):
    total_derivative = 0
    derivatives = []
    
    for i in range(num_samples):
        t = np.random.uniform(0, 1)
        x = w(t, sigma)
        derivative = x / t
        total_derivative += derivative
        derivatives.append(derivative)
    
    average_derivative = total_derivative / num_samples
    
    return average_derivative, derivatives

sigma = 0.5
num_samples = 30000

# Función x = 0 + w(t)
t_values = np.linspace(0.01, 1, 100)  # Evitar división por cero

plt.figure(figsize=(10, 5))

for i in range(5):
    average_derivative, derivatives = monte_carlo_derivative(sigma, num_samples)
    x_values = [w(t, sigma) for t in t_values]
    plt.plot(t_values, x_values, label=f'Iteración {i+1}')

plt.xlabel('Tiempo')
plt.ylabel('Valor de x')
plt.title('Función estocástica con 5 iteraciones')
plt.legend()

plt.tight_layout()
plt.show()
