import numpy as np

class Control_Cost():
    def __init__(self, prob):
        self.prob = prob

    def eval_f(self, X):
        prob = self.prob
        n = prob["n"]
        qdim = prob["qdim"]
        udim = prob["udim"]
        x_2d = X.reshape(n, 2*qdim+udim)
        u = x_2d[:, 2*qdim:]
        value = np.sum(np.power(u, 2))
        return value


    def eval_grad_f(self, X):
        prob = self.prob
        n = prob["n"]
        qdim = prob["qdim"]
        udim = prob["udim"]
        x_2d = X.reshape(n, 2*qdim+udim)
        u = x_2d[:, 2*qdim:]

        grad_2d = np.zeros_like(x_2d)
        grad_2d[:, 2*qdim:] = u
        grad = grad_2d.flatten()
        return grad

