import numpy as np
import matplotlib.pyplot as plt

def polyfit_approximation(x, y, degree):
    # Calculate the coefficients of the polynomial that fits the data
    coeffs = np.polyfit(x, y, degree)
    # Generate a polynomial function using the coefficients
    poly_func = np.poly1d(coeffs)
    return poly_func, coeffs

def plot_approximation(x, y, poly_func):
    # Plot the original data points
    plt.plot(x, y, 'o', label='Original data')

    # Plot the polynomial approximation
    x_new = np.linspace(x[0], x[-1], 1000)
    y_new = poly_func(x_new)
    plt.plot(x_new, y_new, label='Polynomial approximation')

    # Add axis labels and legend
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()

    # Show the plot
    plt.show()

# Define the x and y arrays
def f(x):
    return np.exp(x)

#we can sub in our x and y array here if we want to
x = np.linspace(0, 10, 100)
y = f(x)

# Calculate the polynomial regression approximation 
degree = 10
poly_func, coeffs = polyfit_approximation(x, y, degree)

# Display the polynomial function
poly_str = 'Polynomial function: '
for i, c in enumerate(coeffs):
    if i == 0:
        poly_str += f'{c:.2f}'
    else:
        poly_str += f' + {c:.2f}x^{i}'
print(poly_str)

# Plot the results
plot_approximation(x, y, poly_func)
