import numpy as np


def vectorize(angle):
    """Convert an angle to a representative unit vector.

    Parameters
    ----------
    angle : float
        Angle in radians

    Returns
    -------
    np.ndarray
        Vector
    """
    return np.array((np.cos(angle), np.sin(angle)))


def turn(p1, p2):
    """Compute the vector turn value from origin→p1 to origin→p2.

    This value is positive if a left turn is the fastest way to go from p1 to p2,
    zero if p1 and p2 are colinear, and negative otherwise

    Parameters
    ----------
    p1 : tuple | np.ndarray
        Point
    p2 : tuple | np.ndarray
        Point

    Returns
    -------
    float
    """
    return p1[0] * p2[1] - p2[0] * p1[1]


def project(a, b):
    """Project vector a onto vector b.

    Parameters
    ----------
    a : tuple | np.ndarray
        Vector
    b : tuple | np.ndarray
        Vector

    Returns
    -------
    np.ndarray
    """
    return b * (np.dot(a, b) / np.dot(b, b))


def line_circle_intersect(line, center, radius):
    """Determine if the line in the direction of the first argument intersects the
    circle defined by the second and third arguments.

    Parameters
    ----------
    line : tuple | np.ndarray
        Line
    center : tuple | np.ndarray
        Center of circle
    radius : float
        Radius of circle

    Returns
    -------
    bool
        True if the line intersects the circle, False otherwise.
    """
    clDiffVec = center - project(center, line)
    return np.dot(clDiffVec, clDiffVec) <= radius**2


def fast_pairwise_distances(points, collapse_diagonal_along=None) -> np.ndarray:
    """Compute the pairwise distances between points in a 2D array.

    Parameters
    ----------
    points : np.ndarray
        An array of shape (n, 2) where n is the number of points.

    Returns
    -------
    np.ndarray
        An array of shape (n, n) where n is the number of points.

    See Also
    --------
    https://www.ancisoft.com/blog/using-numpy-to-find-the-average-distance-in-a-set-of-points/
    """
    points = np.asarray(points)
    n = points.shape[0]
    norms_sq = np.sum(points**2, axis=1)
    dist_sq = norms_sq[:, np.newaxis] + norms_sq - 2 * np.dot(points, points.T)
    dist_sq = np.maximum(dist_sq, 0.0)  # Avoid negative values from precision errors
    distances = np.sqrt(dist_sq)
    if collapse_diagonal_along is not None:
        shape = (n, -1) if collapse_diagonal_along == 0 else (-1, n)
        distances = distances[~np.eye(n, dtype=bool)].reshape(shape)
    return distances
