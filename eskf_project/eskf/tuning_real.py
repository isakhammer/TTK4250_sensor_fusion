import numpy as np
from quaternion import RotationQuaterion
from datatypes.eskf_params import ESKFTuningParams
from datatypes.eskf_states import NominalState, ErrorStateGauss

tuning_params_real = ESKFTuningParams(
    accm_std=0.07 * (1.0 / 60.0),                   #m/s/sqrt(h) --> m/s/60*sqrt(sec)
    accm_bias_std=0.05 * (9.81 / 100.0),            #mg --> 9.81/1000 m/s^2
    accm_bias_p=0.000001,

    gyro_std=0.15 * (np.pi / (180.0 * 60.0)),         #deg/sqrt(h) --> pi/180/60 rad/sqrt(sec)
    gyro_bias_std=0.3 * (np.pi / (180.0 * 360.0)),     #deg/h --> pi/180/3600
    gyro_bias_p=0.000001,

    gnss_std_ne=0.0921,
    gnss_std_d=0.2693)

x_nom_init_real = NominalState(
    np.array([0.0, 0.0, 0.0]),  # position
    np.array([20.0, 0.0, 0.0]),  # velocity
    RotationQuaterion.from_euler([0.0, 0.0, 0.0]),  # orientation
    np.array([0.0, 0.0, 0.0]),  # accelerometer bias
    np.array([0.0, 0.0, 0.0]),  # gyro bias
    ts=0.)

init_std_real = np.repeat(repeats=3,  # repeat each element 3 times
                          a=[1.,  # position
                             1.,  # velocity
                             np.deg2rad(15),  # angle vector
                             0.05,  # accelerometer bias
                             0.0001])  # gyro bias

x_err_init_real = ErrorStateGauss(np.zeros(15), np.diag(init_std_real**2), 0.)
