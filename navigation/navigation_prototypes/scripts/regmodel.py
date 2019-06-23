#!/usr/bin/env python
import numpy as np
from scipy.stats import norm
from scipy.optimize import minimize


class EMModel:
    def __init__(self, x, w, e):
        self.x = x
        self.w = w
        self.e = e

    def objfun(self, w):
        return -norm.logpdf(self.e,
                            scale=np.sqrt(np.exp(np.dot(self.x, w)))).sum()

    def gradfun(self, w):
        return -np.dot(self.e**2 * np.exp(-np.dot(self.x, w)) - 1, self.x) / 2

    def run(self):
        return minimize(self.objfun, self.w, jac=self.gradfun)


def main():
    x = np.random.uniform(0, 1, int(1e5))
    x = np.stack([np.ones_like(x), x]).T
    w = np.array([1, 9])
    e = np.random.normal(0, 1, x.shape[0]) * \
        np.sqrt(np.exp(-4.9 + 5 * x[:, 1]))
    test = EMModel(x, w, e)
    return test.run()


if __name__ == '__main__':
    print(main())
