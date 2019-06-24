#!/usr/bin/env python
import numpy as np
from scipy.stats import norm
from scipy.optimize import minimize


def objfun(w, *args):
    x, e = args
    return -norm.logpdf(e, scale=np.sqrt(np.exp(np.dot(x, w)))).sum()


def gradfun(w, *args):
    x, e = args
    return -np.dot(e**2 * np.exp(-np.dot(x, w)) - 1, x) / 2


def maxweights(x, e, w):
    return minimize(objfun, w, (x, e))


def main():
    x = np.random.uniform(0, 1, int(1e5))
    x = np.stack([np.ones_like(x), x]).T
    w = np.array([1, 9])
    e = np.random.normal(0, 1, x.shape[0]) * \
        np.sqrt(np.exp(-4.9 + 5 * x[:, 1]))
    test = maxweights(x, e, w)
    return test


if __name__ == '__main__':
    print(main())
