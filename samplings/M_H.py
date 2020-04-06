import numpy as np 
import matplotlib.pyplot as plt
import scipy.stats as st

def Q(x):
    """
    The function of a random distribution
    Used to represent the Markov Transform Matrix
    Calculating Q(x -> y)
    
    """
    return np.random.normal(loc = x)

def P(x):
    """
    A Probability Distribution to be sampled
    Here we want a Gaussian Distribution
    
    """
    return st.norm.pdf(x, loc=0, scale=1)

def Metropolis_Hasting(iter):
    """
    Metropolis Hasting Algorithm 
    
    """
    xt = 0.0 # First random guess for x
    sample = []

    for i in range(iter):
        x_next = Q(xt)
        alpha = min(1, P(x_next) / P(xt))

        u = np.random.rand()
        if (u <= alpha):
            xt = x_next
        sample.append(xt)

    return sample

if __name__ == '__main__':
    samples = Metropolis_Hasting(10000)
    compare = data = np.random.randn(10000)
    plt.subplot(121)
    plt.hist(samples,  bins=40, normed=0, facecolor="blue", edgecolor="black", alpha=0.7)
    plt.subplot(122)
    plt.hist(compare,  bins=40, normed=0, facecolor="blue", edgecolor="black", alpha=0.7)
    plt.show()