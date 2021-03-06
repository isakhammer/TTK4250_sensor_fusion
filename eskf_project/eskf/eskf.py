import numpy as np
from numpy import ndarray
import scipy
from dataclasses import dataclass, field
from typing import Tuple
from functools import cache

from datatypes.multivargaussian import MultiVarGaussStamped
from datatypes.measurements import (ImuMeasurement,
                                    CorrectedImuMeasurement,
                                    GnssMeasurement)
from datatypes.eskf_states import NominalState, ErrorStateGauss
from utils.indexing import block_3x3

from quaternion import RotationQuaterion
from cross_matrix import get_cross_matrix

import solution


@dataclass
class ESKF():

    accm_std: float
    accm_bias_std: float
    accm_bias_p: float

    gyro_std: float
    gyro_bias_std: float
    gyro_bias_p: float

    gnss_std_ne: float
    gnss_std_d: float

    accm_correction: 'ndarray[3,3]'
    gyro_correction: 'ndarray[3,3]'
    lever_arm: 'ndarray[3]'

    do_approximations: bool
    use_gnss_accuracy: bool = False

    Q_err: 'ndarray[12,12]' = field(init=False, repr=False)
    g: 'ndarray[3]' = np.array([0, 0, 9.82])

    def __post_init__(self):

        self.Q_err = scipy.linalg.block_diag(
            self.accm_std ** 2 * self.accm_correction @ self.accm_correction.T,
            self.gyro_std ** 2 * self.gyro_correction @ self.gyro_correction.T,
            self.accm_bias_std ** 2 * np.eye(3),
            self.gyro_bias_std ** 2 * np.eye(3),
        )
        self.gnss_cov = np.diag([self.gnss_std_ne]*2 + [self.gnss_std_d])**2

    def correct_z_imu(self,
                      x_nom_prev: NominalState,
                      z_imu: ImuMeasurement,
                      ) -> CorrectedImuMeasurement:
        """Correct IMU measurement so it gives a measurmenet of acceleration  
        and angular velocity in body.

        Hint: self.accm_correction and self.gyro_correction translates 
        measurements from IMU frame (probably not correct name) to body frame

        Args:
            x_nom_prev (NominalState): previous nominal state
            z_imu (ImuMeasurement): raw IMU measurement

        Returns:
            CorrectedImuMeasurement: corrected IMU measurement
            """

        z_corr = CorrectedImuMeasurement(z_imu.ts,z_imu.acc,z_imu.avel)

        z_corr.acc = self.accm_correction @ (z_imu.acc - x_nom_prev.accm_bias)
        z_corr.avel = self.gyro_correction @ (z_imu.avel - x_nom_prev.gyro_bias)

        # accm or gyro
        # TODO replace this with your own code
        # z_corr = solution.eskf.ESKF.correct_z_imu(self, x_nom_prev, z_imu)

        return z_corr

    def predict_nominal(self,
                        x_nom_prev: NominalState,
                        z_corr: CorrectedImuMeasurement,
                        ) -> NominalState:
        '''Predict the nominal state, given a corrected IMU measurement

        Hint: Discrete time prediction of equation (10.58)
        See the assignment description for more hints 

        Args:
            x_nom_prev (NominalState): previous nominal state
            z_corr (CorrectedImuMeasurement): corrected IMU measuremnt

        Returns:
            x_nom_pred (NominalState): predicted nominal state
        '''

        #Define a and omega according to task
        R = x_nom_prev.ori.as_rotmat()
        acc_approx = R @ (z_corr.acc) + self.g
        omega = z_corr.avel 

        #Time step
        T_s = z_corr.ts - x_nom_prev.ts

        #Calc predicted nominal states
        #Position
        p_next = x_nom_prev.pos + T_s * x_nom_prev.vel + (T_s**2) * acc_approx / 2

        #Velocity
        v_next = x_nom_prev.vel + T_s * acc_approx

        #Quaternion
        omega_norm = np.linalg.norm(omega)
        kappa = omega/omega_norm
        alpha = T_s * omega_norm  #From task


        #Calc new quaternion according to task
        q_temp_real_part = np.cos(alpha / 2)
        q_temp_vec_part = np.sin(alpha / 2) * kappa

        #Calc prediction
        q_temp = RotationQuaterion(q_temp_real_part,q_temp_vec_part)
        q_next = x_nom_prev.ori @ q_temp
        #q_next = q_next / np.linalg.norm(q_next)

        #Bias predictions
        # next_accm_bias = x_nom_prev.accm_bias + T_s * (-self.accm_bias_p * np.eye(3) @ x_nom_prev.accm_bias)# xdot = px  -> x = e ^-tp  x_0  => xk1 = e^dt*p xk
        # next_gyro_bias = x_nom_prev.gyro_bias + T_s * (-self.gyro_bias_p * np.eye(3) @ x_nom_prev.gyro_bias)# 
        next_accm_bias = np.exp( -T_s*self.accm_bias_p)* x_nom_prev.accm_bias
        next_gyro_bias = np.exp( -T_s*self.gyro_bias_p)* x_nom_prev.gyro_bias 

        #Create new NominalState
        x_nom_pred = NominalState(p_next, v_next, q_next, next_accm_bias, next_gyro_bias, z_corr.ts)
        # g?? videre til neste oppgave 

        return x_nom_pred

    def get_error_A_continous(self,
                              x_nom_prev: NominalState,
                              z_corr: CorrectedImuMeasurement,
                              ) -> 'ndarray[15,15]':
        """Get the transition matrix, A, in (10.68)

        Hint: The S matrices can be created using get_cross_matrix. In the book
        a perfect IMU is expected (thus many I matrices). Here we have 
        to use the correction matrices, self.accm_correction and 
        self.gyro_correction, instead of som of the I matrices.  

        You can use block_3x3 to simplify indexing if you want to.
        The first I element in A can be set as A[block_3x3(0, 1)] = np.eye(3)

        Args:
            x_nom_prev (NominalState): previous nominal state
            z_corr (CorrectedImuMeasurement): corrected IMU measurement
        Returns:
            A (ndarray[15,15]): A
        """

        # Transition transition matrix 10.68
        R_q = x_nom_prev.ori.as_rotmat()
        S_a = get_cross_matrix(z_corr.acc)
        S_w = get_cross_matrix(z_corr.avel)

        # Get dimension of I or use A[block_3x3] = np.eye(3)
        I = np.eye(3) 
        
        # Get p_ab and p_wb parameters
        p_ab = self.accm_bias_p
        p_wb = self.gyro_bias_p

        # A
        A = np.zeros((15,15))
        A[block_3x3(0,1)] = I

        A[block_3x3(1,2)] = -R_q@S_a
        A[block_3x3(1,3)] = -R_q @ self.accm_correction

        A[block_3x3(2,2)] = -S_w
        A[block_3x3(2,4)] = -self.gyro_correction

        A[block_3x3(3,3)] = -p_ab*I
        A[block_3x3(4,4)] = -p_wb*I

        # TODO replace this with your own code
        # A = solution.eskf.ESKF.get_error_A_continous(self, x_nom_prev, z_corr)

        return A

    def get_error_GQGT_continous(self,
                                 x_nom_prev: NominalState
                                 ) -> 'ndarray[15, 12]':
        """The noise covariance matrix, GQGT, in (10.68)

        From (Theorem 3.2.2) we can see that (10.68) can be written as 
        d/dt x_err = A@x_err + G@n == A@x_err + m
        where m is gaussian with mean 0 and covariance G @ Q @ G.T. Thats why
        we need GQGT.

        Hint: you can use block_3x3 to simplify indexing if you want to.
        The first I element in G can be set as G[block_3x3(2, 1)] = -np.eye(3)

        Args:
            x_nom_prev (NominalState): previous nominal state
        Returns:
            GQGT (ndarray[15, 15]): G @ Q @ G.T
        """
        
        # Find G in 10.68
        R_q = x_nom_prev.ori.as_rotmat()
        G = np.zeros((3*5, 3*4))
        I = np.identity(3)

        G[block_3x3(1,0)] = -R_q
        G[block_3x3(2,1)] = -I
        G[block_3x3(3,2)] = I
        G[block_3x3(4,3)] = I

        GQGT = G@self.Q_err@G.T


        # TODO replace this with your own code
        #GQGT = solution.eskf.ESKF.get_error_GQGT_continous(self, x_nom_prev)

        return GQGT

    def get_van_loan_matrix(self, V: 'ndarray[30, 30]'):
        """Use this funciton in get_discrete_error_diff to get the van loan 
        matrix. See (4.63)

        All the tests are ran with do_approximations=False

        Args:
            V (ndarray[30, 30]): [description]

        Returns:
            VanLoanMatrix (ndarray[30, 30]): VanLoanMatrix
        """
        if self.do_approximations:
            # second order approcimation of matrix exponential which is faster
            VanLoanMatrix = np.eye(*V.shape) + V + (V@V) / 2
        else:
            VanLoanMatrix = scipy.linalg.expm(V)
        return VanLoanMatrix

    def get_discrete_error_diff(self,
                                x_nom_prev: NominalState,
                                z_corr: CorrectedImuMeasurement,
                                ) -> Tuple['ndarray[15, 15]',
                                           'ndarray[15, 15]']:
        """Get the discrete equivalents of A and GQGT in (4.63)

        Hint: you should use get_van_loan_matrix to get the van loan matrix

        See (4.5 Discretization) and (4.63) for more information. 
        Or see "Discretization of process noise" in 
        https://en.wikipedia.org/wiki/Discretization

        Args:
            x_nom_prev (NominalState): previous nominal state
            z_corr (CorrectedImuMeasurement): corrected IMU measurement

        Returns:
            Ad (ndarray[15, 15]): discrede transition matrix
            GQGTd (ndarray[15, 15]): discrete noise covariance matrix
        """
        #Get matrices for continous time
        Aerr = self.get_error_A_continous(x_nom_prev,z_corr)
        GQGTerr = self.get_error_GQGT_continous(x_nom_prev)

        #Matrix according to hint
        T_s = z_corr.ts - x_nom_prev.ts #Is this correct!?
        V = np.block([[-Aerr, GQGTerr], [np.zeros_like(Aerr), Aerr.T]]) * T_s

        #Calc exponential with possibility for approx if given (as stated in task)
        Vexp = self.get_van_loan_matrix(V)

        #Extract discrete matrices
        AdT = Vexp[15:,15:]
        Ad = AdT.T

        GQGTd = Ad @ Vexp[0:15,15:]

        return Ad, GQGTd

    def predict_x_err(self,
                      x_nom_prev: NominalState,
                      x_err_prev_gauss: ErrorStateGauss,
                      z_corr: CorrectedImuMeasurement,
                      ) -> ErrorStateGauss:
        """Predict the error state

        Hint: This is doing a discrete step of (10.68) where x_err 
        is a multivariate gaussian.

        Args:
            x_nom_prev (NominalState): previous nominal state
            x_err_prev_gauss (ErrorStateGauss): previous error state gaussian
            z_corr (CorrectedImuMeasurement): corrected IMU measuremnt

        Returns:
            x_err_pred (ErrorStateGauss): predicted error state
        """
        
        #Get system matrices
        Ad, GQGTd = self.get_discrete_error_diff(x_nom_prev,z_corr)

        #Mean is zero
        predict_x_err_mean = np.zeros((15,))

        #Calc covariance matrix for discrete case
        predict_x_err_cov = Ad @ x_err_prev_gauss.cov @ Ad.T + GQGTd

        #Time stamp
        predict_x_err_ts = z_corr.ts

        #New ErrorStateGaussian
        x_err_pred = ErrorStateGauss(predict_x_err_mean,predict_x_err_cov,predict_x_err_ts)

        return x_err_pred

    def predict_from_imu(self,
                         x_nom_prev: NominalState,
                         x_err_gauss: ErrorStateGauss,
                         z_imu: ImuMeasurement,
                         ) -> Tuple[NominalState, ErrorStateGauss]:
        """Method called every time an IMU measurement is received

        Args:
            x_nom_prev (NominalState): previous nominal state
            x_err_gauss (ErrorStateGauss): previous error state gaussian
            z_imu (ImuMeasurement): raw IMU measurement

        Returns:
            x_nom_pred (NominalState): predicted nominal state
            x_err_pred (ErrorStateGauss): predicted error state
        """

        #Correct the imu-measurement
        z_corr = self.correct_z_imu(x_nom_prev,z_imu)

        #Call the two finished functions
        x_nom_pred = self.predict_nominal(x_nom_prev,z_corr)
        x_err_pred = self.predict_x_err(x_nom_prev,x_err_gauss,z_corr)
        
        # TODO replace this with your own code
        #x_nom_pred, x_err_pred = solution.eskf.ESKF.predict_from_imu(
           # self, x_nom_prev, x_err_gauss, z_imu)

        return x_nom_pred, x_err_pred

    def get_gnss_measurment_jac(self, x_nom: NominalState) -> 'ndarray[3,15]':
        """Get the measurement jacobian, H.

        Hint: the gnss antenna has a relative position to the center given by
        self.lever_arm. How will the gnss measurement change if the drone is 
        rotated differently? Use get_cross_matrix and some other stuff :)

        Returns:
            H (ndarray[3, 15]): [description]
        """

        #Preallocate
        H = np.zeros((3,15))

        #Top part of H is identity (Eq. 10.80 <in book)
        H[0:3,0:3] = np.eye(3)

        #Using hint the rest of H is not zero as in Eq. 10.80, but rather accounts for the rotation of the lever_arm
        H[0:3,6:9] = -x_nom.ori.R @ get_cross_matrix(self.lever_arm)

        return H

    def get_gnss_cov(self, z_gnss: GnssMeasurement) -> 'ndarray[3,3]':
        """Use this function in predict_gnss_measurement to get R. 
        Get gnss covariance estimate based on gnss estimated accuracy. 

        All the test data has self.use_gnss_accuracy=False, so this does not 
        affect the tests.

        There is no given solution to this function, feel free to play around!

        Returns:
            gnss_cov (ndarray[3,3]): the estimated gnss covariance
        """
        if self.use_gnss_accuracy and z_gnss.accuracy is not None:
            # play around with this part, the suggested way is not optimal
            gnss_cov = (z_gnss.accuracy/3)**2 * self.gnss_cov

        else:
            # dont change this part
            gnss_cov = self.gnss_cov
        return gnss_cov

    def predict_gnss_measurement(self,
                                 x_nom: NominalState,
                                 x_err: ErrorStateGauss,
                                 z_gnss: GnssMeasurement,
                                 ) -> MultiVarGaussStamped:
        """Predict the gnss measurement

        Hint: z_gnss is only used in get_gnss_cov and to get timestamp for 
        the predicted measurement

        Args:
            x_nom (NominalState): previous nominal state
            x_err (ErrorStateGauss): previous error state gaussian
            z_gnss (GnssMeasurement): gnss measurement

        Returns:
            z_gnss_pred_gauss (MultiVarGaussStamped): gnss prediction gaussian
        """

        #Mean
        z_gnss_pred_gauss_mean = x_nom.pos + x_nom.ori.R @ self.lever_arm

        #Get jacobian and cov matrices
        H = self.get_gnss_measurment_jac(x_nom)
        gnss_cov = self.get_gnss_cov(z_gnss)

        #Covariance
        z_gnss_pred_gauss_cov = H @ x_err.cov @ H.T + gnss_cov * np.eye(3)

        #Get time stamp
        z_gnss_pred_gauss_ts = z_gnss.ts

        #New MultiVarGaussStamped
        z_gnss_pred_gauss = MultiVarGaussStamped(z_gnss_pred_gauss_mean, z_gnss_pred_gauss_cov, z_gnss_pred_gauss_ts)

        return z_gnss_pred_gauss

    def get_x_err_upd(self,
                      x_nom: NominalState,
                      x_err: ErrorStateGauss,
                      z_gnss_pred_gauss: MultiVarGaussStamped,
                      z_gnss: GnssMeasurement
                      ) -> ErrorStateGauss:
        """Update the error state from a gnss measurement

        Hint: see (10.75)
        Due to numerical error its recomended use the robust calculation of 
        posterior covariance.

        I_WH = np.eye(*P.shape) - W @ H
        P_upd = (I_WH @ P @ I_WH.T + W @ R @ W.T)

        Args:
            x_nom (NominalState): previous nominal state
            x_err (ErrorStateGauss): previous error state gaussian
            z_gnss_pred_gauss (MultiVarGaussStamped): gnss prediction gaussian
            z_gnss (GnssMeasurement): gnss measurement

        Returns:
            x_err_upd_gauss (ErrorStateGauss): updated error state gaussian
        """
        
        #Follow hint: Need to calc W accordint to Eq. 10.75
        P = x_err.cov
        H = self.get_gnss_measurment_jac(x_nom)
        R = self.get_gnss_cov(z_gnss)

        W = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)

        #Use robust method from hint
        I_WH = np.eye(*P.shape) - W @ H
        P_upd = (I_WH @ P @ I_WH.T + W @ R @ W.T)

        #Update mean
        mean_upd = W @ (z_gnss.pos - z_gnss_pred_gauss.mean)

        #Time stamp
        ts_x_err_upd = z_gnss.ts

        #New ErrorStateGauss
        x_err_upd_gauss = ErrorStateGauss(mean_upd, P_upd, ts_x_err_upd)

        return x_err_upd_gauss

    def inject(self,
               x_nom_prev: NominalState,
               x_err_upd: ErrorStateGauss
               ) -> Tuple[NominalState, ErrorStateGauss]:
        """Perform the injection step

        Hint: see (10.85) and (10.72) on how to inject into nominal state.
        See (10.86) on how to find error state after injection

        Args:
            x_nom_prev (NominalState): previous nominal state
            x_err_upd (ErrorStateGauss): updated error state gaussian

        Returns:
            x_nom_inj (NominalState): nominal state after injection
            x_err_inj (ErrorStateGauss): error state gaussian after injection
        """

        #Calc injection step according to Eq. 10.72 in book
        pos_inject = x_nom_prev.pos + x_err_upd.pos
        vel_inject = x_nom_prev.vel + x_err_upd.vel
        ori_inject = x_nom_prev.ori @ RotationQuaterion(1, x_err_upd.avec / 2)
        accm_bias_inject = x_nom_prev.accm_bias + x_err_upd.accm_bias
        gyro_bias_inject = x_nom_prev.gyro_bias + x_err_upd.gyro_bias
        
        #Time stamp
        ts_inject = x_err_upd.ts

        #New nominal state
        x_nom_inj = NominalState(pos_inject, vel_inject, ori_inject, accm_bias_inject,gyro_bias_inject, ts_inject)

        #Follow the hint, and do as stated in Eq. 10.86 in book
        #Calc G matrix
        center_of_matrix = np.eye(3) - get_cross_matrix(x_err_upd.avec / 2)
        G = scipy.linalg.block_diag(np.eye(6), center_of_matrix, np.eye(6))

        #Find new P matrix
        P_inject = G @ x_err_upd.cov @ G.T

        x_err_inj = ErrorStateGauss(np.zeros((15, )), P_inject, ts_inject)

        return x_nom_inj, x_err_inj

    def update_from_gnss(self,
                         x_nom_prev: NominalState,
                         x_err_prev: ErrorStateGauss,
                         z_gnss: GnssMeasurement,
                         ) -> Tuple[NominalState,
                                    ErrorStateGauss,
                                    MultiVarGaussStamped]:
        """Method called every time an gnss measurement is received.


        Args:
            x_nom_prev (NominalState): previous nominal state
            x_err_prev (ErrorStateGauss): previous error state gaussian
            z_gnss (GnssMeasurement): gnss measurement

        Returns:
            x_nom_inj (NominalState): nominal state after injection
            x_err_inj (ErrorStateGauss): error state gaussian after injection
            z_gnss_pred_gauss (MultiVarGaussStamped): predicted gnss 
                measurement, used for NIS calculations.
        """

        #Look at data types, and call functions in appropriate order
        z_gnss_pred_gauss = self.predict_gnss_measurement(x_nom_prev,x_err_prev,z_gnss)

        x_err_upd = self.get_x_err_upd(x_nom_prev,x_err_prev,z_gnss_pred_gauss,z_gnss)

        x_nom_inj, x_err_inj = self.inject(x_nom_prev,x_err_upd)

        return x_nom_inj, x_err_inj, z_gnss_pred_gauss
