import numpy as np
from pathlib import Path

import typing as t


def read_from_csv(filename: t.Union[str, Path]):
    if isinstance(filename, str):
        filename = Path(filename)
        # should assert is a file
    if not filename.exists():
        raise FileNotFoundError(f"File {filename.absolute()} does not exist")

    return np.genfromtxt(filename, delimiter=",")


def polar_to_cartesian(r, theta):
    finite_mask = np.isfinite(r)
    # filter out infinities
    r = r[finite_mask]
    theta = theta[finite_mask]

    x = r * np.cos(theta)
    y = r * np.sin(theta)
    # return [2, x] output for easy plotting
    return np.vstack([x, y])


def cartesian_to_polar(x, y):
    r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y, x)
    # return [2, x] output for easy plotting
    return np.hstack([theta, r]).T
