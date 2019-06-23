import numpy as np
from scipy.optimize import minimize
from scipy.stats import norm


class Heteroskedastic:
    def __init__(self, x_i, w, e):
        self.x1 = x_i
        self.X = np.concatenate([np.ones((10000, 1)), self.x1], axis=1)
        self.w = w
        self.e = e

    def obj_fun(self, w):
        loglik = 0
        for i in range(self.X.shape[0]):
            variance = np.exp(np.matmul(self.X[i], w))
            loglik += -norm.logpdf(self.e[i], loc=0, scale=np.sqrt(variance))
        return loglik

    def grad_fun(self, w):
        grad = np.zeros((w.shape[0], 1))
        for i in range(self.e.shape[0]):
            temp1 = (np.exp(np.matmul(-w.transpose(),
                                      (self.X[i])[np.newaxis].transpose())))[np.newaxis]
            temp2 = (1 - ((self.e[i])**2)*temp1)
            grad += 0.5*np.matmul((self.X[i])[np.newaxis].transpose(), temp2)
        return grad.ravel()

    def minimize_fun(self, obj_fun, grad_fun, w):
        res = minimize(obj_fun, w, method='BFGS',
                       jac=grad_fun, options={'disp': True})
        return res

    def run(self):
        res = self.minimize_fun(self.obj_fun, self.grad_fun, self.w)
        print res.x
        print self.grad_fun(res.x)
        print "result sd:", np.sqrt(np.exp(np.matmul(res.x, self.X.transpose())))
        return res.x


if __name__ == "__main__":
    x = np.random.uniform(0, 1, (10000, 1))  # other noise/features
    w = np.array([0.1, 0.2])
    e = np.random.normal(0, 1, (10000, 1)) * np.sqrt(np.exp(-4 + 5*x))
    print("sd:%f", np.std(e))
    test = Heteroskedastic(x, w, e)
    test.run()
