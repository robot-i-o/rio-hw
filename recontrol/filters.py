import numbers

import numpy as np


class LowPassFilter:
    def __init__(self, alpha: np.ndarray | numbers.Number, initial=0.0):
        assert ((0 < np.array(alpha)) & (np.array(alpha) < 1)).all()
        self.alpha = alpha
        self.s = initial

    @staticmethod
    def filter(x, s_prev, alpha):
        return alpha * x + (1 - alpha) * s_prev

    def __call__(self, x: np.ndarray | numbers.Number):
        self.s = self.filter(x, self.s, self.alpha)
        if isinstance(x, np.ndarray):
            return self.s.copy()
        return self.s
