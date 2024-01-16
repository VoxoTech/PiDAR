from ulab import numpy as np    # type: ignore


def hstack(*args):
    return np.concatenate(tuple(arg.reshape((-1,1)) for arg in args), axis=-1)

def vstack(*args):
    return np.concatenate(args, axis=0)
