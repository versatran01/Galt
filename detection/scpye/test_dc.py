# -*- coding: utf-8 -*-
"""
Created on Tue Feb  2 10:26:03 2016

@author: chao
"""

def handle_list(func):
    def func_wrapper(X, y=None):
        if y is None:
            if isinstance(X, list):
                return [func(_X) for _X in X]
            else:
                return func(X)
        else:
            # Hopefully the input of y won't be a list of None
            if isinstance(X, list):
                assert isinstance(y, list)
                Xts = []
                yts = []
                for _X, _y in zip(X, y):
                    Xt, yt = func(_X, _y)
                    Xts.append(Xt)
                    yts.append(yt)
                return Xts, yts
            else:
                return func(X, y)
    return func_wrapper

@handle_list
def f(a, b=None):
    if b is None:
        return a + 1
    else:
        return a + 1, b + 2

print(f(1))
print('===')
print(f([1, 2]))
print('===')
print(f(1, 1))
print('===')
print(f([1, 2], [1, 2]))
print('===')