from matplotlib import pyplot as plt
import numpy as np

def noisySignal(x,f):
    """Generate a noisy signal."""
    noise = np.random.randint(-20, 20)
    return f(x) + noise

def noisyArray(x, f):
    """Generate an array of noisy signals."""
    return [noisySignal(xi, f) for xi in x]

def plotSignal(x):
    """Plot the noisy signal."""
    f = lambda x: 0.5 * x**2
    y_noisy = noisyArray(x, f)
    y = [f(xi) for xi in x]

    plt.plot(x, y_noisy, label='Noisy Signal', color='blue', linestyle='--')
    plt.plot(x, y, label='Original Signal', color='orange')
    plt.title('Noisy Signal Plot')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.grid()
    plt.show()
plotSignal(np.arange(-10, 11, 1))


