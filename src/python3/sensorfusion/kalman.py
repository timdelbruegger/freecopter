from numpy import dot, sum, tile, exp, log, pi, shape, reshape
from numpy.linalg import inv, pinv, LinAlgError, det
import logging

logger = logging.getLogger("KalmanFilter")


# X: state vector at k-1
# P: covariance matrix at k-1
# A: state transition matrix
# Q: process noise covariance matrix
# B: input effect matrix
# U: control input vector
def kf_predict(x, P, A, Q, B, u):

    shape_X_before = shape(x)

    logger.debug("A: %s", A)
    logger.debug("B: %s", B)
    logger.debug("x: %s", x)
    logger.debug("u: %s", u)

    logger.debug("shape A: %s", A.shape)
    logger.debug("shape B: %s", B.shape)
    logger.debug("shape x: %s", x.shape)
    logger.debug("shape u: %s", u.shape)

    assert(shape(u)[0] == B.shape[1])
    assert(A.shape[1] == B.shape[0])

    Ax = dot(A, x)
    Bu = dot(B, u)

    Bu = reshape(Bu, (shape(Ax)))

    logger.debug("dot(A, x): %s", Ax)
    logger.debug("dot(B, u): %s", Bu)

    x = Ax + Bu

    logger.debug("x: %s", x)

    P = dot(A, dot(P, A.T)) + Q

    # make sure that we did not change the shape
    # if we do, the matrix dimensions are probably wrong!
    assert(shape(x) == shape_X_before)

    return x, P


# X: state vector at k-1
# P: covariance matrix at k-1
# Y: measurement vector
# H: measurement prediction matrix
# R: measurement noise covariance matrix
def kf_update(X, P, Y, H, R):

    # mean of predictive distribution of Y
    IM = dot(H, X)

    # Covariance or predictive mean of Y
    IS = R + dot(H, dot(P, H.T))

    # Kalman Gain
    try:
        K = dot(P, dot(H.T, pinv(IS)))
    except LinAlgError:
        logger.error("LinAlgError on IS inversion (Kalman Gain)")
        logger.error(IS)
        raise

    #logger.debug("Kalman Gain G")
    #logger.debug(K)

    # update X, P
    X = X + dot(K, (Y-IM))
    P = P - dot(K, dot(IS, K.T))

    #logger.debug(["P in update", P])

    # predictive probability (likelihood) of measurement
    #LH =gauss_pdf(Y, IM, IS)

    return X, P, K, IM, IS #, LH


def gauss_pdf(X, M, S):
    if M.shape()[1] == 1:
        DX = X - tile(M, X.shape()[1])
        E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
        E = E + 0.5 * M.shape()[0] * log(2 * pi) +0.5 * log(det(S))
        P = exp(-E)
    elif X.shape()[1] == 1:
        DX = tile(X, M.shape()[1])- M
        E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
        E = E + 0.5 * M.shape()[0] * log(2 * pi) +0.5 * log(det(S))
        P = exp(-E)
    else:
        DX = X-M
        E = 0.5 * dot(DX.T, dot(inv(S), DX))
        E = E + 0.5 * M.shape()[0] * log(2 * pi) + .5 * log(det(S))
        P = exp(-E)
    return (P[0],E[0])


class KalmanFilter:

    # X: state vector at k-1
    # P: covariance matrix at k-1
    # A: state transition matrix (function for dt)
    # Q: process noise covariance matrix
    # B: control influence (function for dt)
    # H: measurement prediction matrix
    def __init__(self, x, P, A, Q, B, H):
        self.x = x
        self.P = P
        self.A = A
        self.Q = Q
        self.B = B
        self.H = H 

    # U: control input vector
    def predictWithInput(self, U, dt):
        (self.x, self.P) = kf_predict(self.x, self.P, self.A(dt), dt * self.Q, self.B(dt), U)
        return self.x, self.P

    # Y: measurement vector
    # R: measurement noise covariance matrix
    def updateWithMeasurement(self, Y, R):
        (self.x, self.P, K, IM, IS) = kf_update(self.x, self.P, Y, self.H, R)
        return self.x, self.P
