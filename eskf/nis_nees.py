import numpy as np
from numpy import ndarray
from typing import Sequence, Optional

from datatypes.measurements import GnssMeasurement
from datatypes.eskf_states import NominalState, ErrorStateGauss
from datatypes.multivargaussian import MultiVarGaussStamped

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

    measurement = z_gnss.pos

    #Take into account marginal cases
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
    xt = np.hstack([ x_true.pos, x_true.vel, x_true.ori.vec_part, x_true.accm_bias, x_true.gyro_bias ])
    xn = np.hstack([ x_nom.pos, x_nom.vel, x_nom.ori.vec_part, x_nom.accm_bias, x_nom.gyro_bias ])
    error = xt - xn
    

    # TODO replace this with your own code
    error = solution.nis_nees.get_error(x_true, x_nom)

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

    Pkinv = np.linalg.inv( x_err.marginalize(marginal_idxs).cov) 
    NEES = error[marginal_idxs].T @ Pkinv@ error[marginal_idxsj

    # TODO replace this with your own code
    # NEES = solution.nis_nees.get_NEES(error, x_err, marginal_idxs)

    return NEES


def get_time_pairs(unique_data, data):
    """match data from two different time series based on timestamps"""
    gt_dict = dict(([x.ts, x] for x in unique_data))
    pairs = [(gt_dict[x.ts], x) for x in data if x.ts in gt_dict]
    times = [pair[0].ts for pair in pairs]
    return times, pairs
