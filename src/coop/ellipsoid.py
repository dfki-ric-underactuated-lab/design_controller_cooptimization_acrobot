import numpy as np
from scipy.special import gamma


def directSphere(d, r_i=0, r_o=1):
    """
    Implementation: Krauth, Werner. Statistical Mechanics: Algorithms and
    Computations. Oxford Master Series in Physics 13. Oxford: Oxford University
    Press, 2006. page 42
    """
    # vector of univariate gaussians:
    rand = np.random.normal(size=d)
    # get its euclidean distance:
    dist = np.linalg.norm(rand, ord=2)
    # divide by norm
    normed = rand/dist

    # sample the radius uniformly from 0 to 1
    rad = np.random.uniform(r_i, r_o**d)**(1/d)
    # the r**d part was not there in the original implementation.
    # I added it in order to be able to change the radius of the sphere
    # multiply with vect and return
    return normed*rad


def quadForm(M, x):
    """
    Helper function to compute quadratic forms such as x^TMx
    """
    return np.dot(x, np.dot(M, x))


def sampleFromEllipsoid(S, rho, rInner=0, rOuter=1):
    lamb, eigV = np.linalg.eigh(S/rho)
    d = len(S)
    xy = directSphere(d, r_i=rInner, r_o=rOuter)  # sample from outer shells
    # transform sphere to ellipsoid
    # (refer to e.g. boyd lectures on linear algebra)
    T = np.linalg.inv(np.dot(np.diag(np.sqrt(lamb)), eigV.T))
    return np.dot(T, xy.T).T


def volEllipsoid(rho, M):
    """
    Calculate the Volume of a Hyperellipsoid Volume of the Hyperllipsoid
    according to
    https://math.stackexchange.com/questions/332391/volume-of-hyperellipsoid/332434
    Intuition: https://textbooks.math.gatech.edu/ila/determinants-volumes.html
    Volume of n-Ball https://en.wikipedia.org/wiki/Volume_of_an_n-ball
    """

    # For a given hyperellipsoid, find the transformation that when applied to
    # the n Ball yields the hyperellipsoid
    lamb, eigV = np.linalg.eigh(M/rho)
    A = np.dot(np.diag(np.sqrt(lamb)), eigV.T)  # transform ellipsoid to sphere
    detA = np.linalg.det(A)

    # Volume of n Ball (d dimensions)
    d = M.shape[0]  # dimension
    volC = (np.pi**(d/2)) / (gamma((d/2)+1))

    # Volume of Ellipse
    volE = volC/detA
    return volE
