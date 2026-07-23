from line_tools.utils import dist_2_line, cartesian_to_polar
import numpy as np

import typing as t

# need to actually calculate rho_variance
def calculate_covariance_matrix(pts: np.ndarray, rho_variance = 1.0):
    # convert back to polar form
    theta, rho = np.vsplit(cartesian_to_polar(pts[:,0], pts[:,1]), 2)

    weights = 1/rho_variance * np.ones_like(theta)

    # Start by calculating numerator of (1) 
    n1 = 0.
    for i in range(len(theta)):
        for j in range(i+1,len(theta)):
            n1 += weights[i] * weights[j] * rho[i] * rho[j] * np.sin(theta[i] + theta[j])
            # add second term for numerator
            np.sum(weights ** -1)

    n2 = 0.
    for i in range(len(theta)):
        wi = weights[i]
        n2 += (wi - np.sum(weights)) * wi * rho[i] ** 2 * np.sin(2 * theta[i])
        # add second term for numerator
    # technically, need to scale by 2/np.sum(weights) ... 
    N = 2/np.sum(weights) * (n1 + n2)

    # calculate denominator of (1)
    d1 = 0.
    for i in range(len(theta)):
        for j in range(i+1,len(theta)):
            d1 += w * rho[i] * rho[j] * np.cos(theta[i] + theta[j])
    
    d2 = 0.
    for i in range(len(theta)):
        wi = weights[i]
        d2 += (wi - np.sum(weights)) * wi * rho[i] ** 2 * np.cos(2 * theta[i])

    D = np.sum(weights) * (d1 + d2)

    # N / D = tan(2*alpha)
    # verify that this is correct 
    alpha = 0.5 * np.arctan2(N, D)

    r = 0.
    for i in range(len(theta)):
        wi = weights[i]
        r += wi*rho[i]*cos(theta[i] - alpha)
    r = r / np.sum(weights)

    xbar_w = 0.
    ybar_w = 0.
    for i in range(len(theta)):
        xbar_w += weigts[i]* rho[i] * np.cos(theta[i])
        ybar_w += weigts[i]* rho[i] * np.sin(theta[i])
    
    xbar_w = xbar_w / np.sum(weigts)
    ybar_w = ybar_w / np.sum(weigts)

    # calculate (9)
    sigma_a2_scale =((D ** 2 + N ** 2)**2) **-1
    # need to calculate sigma^2_rho_i 
    sigma_a2 = 0. 
    for i in range(len(theta)):
        wi = weights[i]
        # technically these can be pre-calculate from jacobian 
        sigma_a2 += (wi**2) * (xbar_w * np.cos(theta[i]) - ybar_w * np.sin(theta[i]) - rho[i] * np.cos(2 * theta[i])) - D * (xbar_w * np.sin(theta[i]) + ybar_w * np.cos(theta[i]) - rho[i]*np.sin(2 * theta[i]))**2 # * sigma^2 _ rho[i]
    
    sigma_a2 = sigma_a2 * sigma_a2_scale

    # numerator and denominator partial derivative terms, also in sigma_a2, 
    A = xbar_w * np.cos(theta) - ybar_w * np.sin(theta) - rho * np.cos(2 *theta)
    B = xbar_w * np.sin(theta) + ybar_w * np.cos(theta) - rho * np.sin(2 *theta)

    # calculate partial derivatives of a w.r.t rho
    delta_a_wrt_delta_P = (weights * (D * B - N * A)) / (D ** 2 + N ** 2)

    # calculate partial derivatives of r w.r.t rho
    delta_r_wrt_delta_p = np.cos(theta-alpha) + (ybar_w * np.cos(alpha) - xbar_w * np.sin(alpha)) * delta_a_wrt_delta_P 
    delta_r_wrt_delta_p /= len(theta)


    # calculate (10)
    sigma_r2 = 0.
    delta_r_constant_term = (ybar_w * np.cos(alpha) - xbar_w * np.sin(alpha)) # from (5)

    np.cos(theta - alpha) + delta_a_wrt_delta_P * delta_r_constant_term

    for i in range(len(theta)):
        sigma_r2 += (weights[i] / np.sum(weights)) * (np.cos(theta[i] - alpha) + delta_a_wrt_delta_P[i] * delta_r_constant_term )** 2 # * variance rho_i 
    

    # calculate (11)
    sigma_ar = np.sum(delta_r_wrt_delta_p * delta_a_wrt_delta_P) # variance of r_i


    return np.array([
        [sigma_a2, sigma_ar],
        [sigma_ar, sigma_r2]
    ])