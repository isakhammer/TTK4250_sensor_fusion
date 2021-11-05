import numpy as np
from quaternion import RotationQuaterion
from datatypes.eskf_params import ESKFTuningParams
from datatypes.eskf_states import NominalState, ErrorStateGauss

tuning_params_sim = ESKFTuningParams(
    accm_std=100.,
    accm_bias_std=1.,
    accm_bias_p=55.,

    gyro_std=100.,
    gyro_bias_std=1.,
    gyro_bias_p=1.,

    gnss_std_ne=0.0921,
    gnss_std_d=0.2693)

x_nom_init_sim = NominalState(
    np.array([0.2, 0.0, -5.0]),  # position
    np.array([20.0, 0.0275, 0.1133]),  # velocity
    RotationQuaterion.from_euler([1.0, 0.0, 0.0]),  # orientation
    np.array([0.0, 0.0, 0.0]),  # accelerometer bias
    np.array([0.0, 0.0, 0.0]),  # gyro bias
    ts=0.)

init_std_sim = np.repeat(repeats=3,  # repeat each element 3 times
                         a=[1.,  # position
                            1.,  # velocity
                            np.deg2rad(1),  # angle vector
                            1.,  # accelerometer bias
                            1.])  # gyro bias
x_err_init_sim = ErrorStateGauss(np.zeros(15), np.diag(init_std_sim**2), 0.)
