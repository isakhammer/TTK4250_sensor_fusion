import numpy as np
from numpy import ndarray
from typing import Sequence, Optional

from datatypes.measurements import GnssMeasurement
from datatypes.eskf_states import NominalState, ErrorStateGauss
from datatypes.multivargaussian import MultiVarGaussStamped
from quaternion import RotationQuaterion

import solution


def get_NIS(z_gnss: GnssMeasurement,
            z_gnss_pred_gauss: MultiVarGaussStamped,
            marginal_idxs: Optional[Sequence[int]] = None
            ) -> float:
    """Calculate NIS

    Args:
        z_gnss (GnssMeasurement): gnss measurement
        z_gnss_pred_gauss (MultiVarGaussStamped): predicted gnss measurement
        marginal_idxs (Optional[Sequence[int]]): Sequence of marginal indexes.
            For example used for calculating NIS in only xy direction.  

    Returns:
        NIS (float): NIS value
    """

    #Get measurement
    measurement = z_gnss.pos

    #Take into account marginalization cases
    if marginal_idxs != None:
        measurement = measurement[marginal_idxs]
        z_gnss_pred_gauss = z_gnss_pred_gauss.marginalize(marginal_idxs)

    #Already done for us
    NIS = z_gnss_pred_gauss.mahalanobis_distance_sq(measurement)
    
    return NIS


def get_error(x_true: NominalState,
              x_nom: NominalState,
              ) -> 'ndarray[15]':
    """Finds the error (difference) between True state and 
    nominal state. See (Table 10.1).


    Returns:
        error (ndarray[15]): difference between x_true and x_nom. 
    """

    #Error position and velocity as in table 10.1
    error_pos = x_true.pos - x_nom.pos
    error_vel = x_true.vel - x_nom.vel

    #Need some trickeru for error of orientation as in Table 10.1
    q = x_nom.ori
    norm_of_q_squared = q.real_part**2 + q.vec_part.T @ q.vec_part
    q_conj = q.conjugate()

    #Inverse of quaternion (Eq. 10.27)
    q_inv_real_part = q_conj.real_part / norm_of_q_squared
    q_inv_vec_part = q_conj.vec_part / norm_of_q_squared
    q_inv = RotationQuaterion(q_inv_real_part,q_inv_vec_part)

    #Error of orientation as in Table 10.1
    error_q = q_inv @ x_true.ori
    error_ori = error_q.as_avec()

    #Error of biases as in table 10.1
    error_accm_bias = x_true.accm_bias - x_nom.accm_bias
    error_gyro_bias = x_true.gyro_bias - x_nom.gyro_bias

    error = np.concatenate((error_pos, error_vel, error_ori, error_accm_bias, error_gyro_bias))

    return error


def get_NEES(error: 'ndarray[15]',
             x_err: ErrorStateGauss,
             marginal_idxs: Optional[Sequence[int]] = None
             ) -> float:
    """Calculate NEES

    Args:
        error (ndarray[15]): errors between x_true and x_nom (from get_error)
        x_err (ErrorStateGauss): estimated error
        marginal_idxs (Optional[Sequence[int]]): Sequence of marginal indexes.
            For example used for calculating NEES for only the position. 

    Returns:
        NEES (float): NEES value
    """

    #Take into account marginalization cases
    if marginal_idxs != None:
        error = error[marginal_idxs]
        x_err = x_err.marginalize(marginal_idxs)

    #Covariance
    P = x_err.cov

    #Calc NEES according to Eq. 4.65 in book
    NEES = error.T @ np.linalg.inv(P) @ error

    return NEES


def get_time_pairs(unique_data, data):
    """match data from two different time series based on timestamps"""
    gt_dict = dict(([x.ts, x] for x in unique_data))
    pairs = [(gt_dict[x.ts], x) for x in data if x.ts in gt_dict]
    times = [pair[0].ts for pair in pairs]
    return times, pairs
