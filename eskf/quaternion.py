import numpy as np
from numpy import ndarray
from dataclasses import dataclass
from scipy.spatial.transform import Rotation
from cross_matrix import get_cross_matrix

from config import DEBUG

import solution


@dataclass
class RotationQuaterion:
    """Class representing a rotation quaternion (norm = 1). Has some useful
    methods for converting between rotation representations.

    Hint: You can implement all methods yourself, or use scipys Rotation class.
    scipys Rotation uses the xyzw notation for quats while the book uses wxyz
    (this i really annoying, I know).

    Args:
        real_part (float): eta (n) in the book, w in scipy notation
        vec_part (ndarray[3]): epsilon in the book, (x,y,z) in scipy notation
    """
    real_part: float
    vec_part: 'ndarray[3]'

    def __post_init__(self):
        if DEBUG:
            assert len(self.vec_part) == 3

        norm = np.sqrt(self.real_part**2 + sum(self.vec_part**2))
        if not np.allclose(norm, 1):
            self.real_part /= norm
            self.vec_part /= norm

        if self.real_part < 0:
            self.real_part *= -1
            self.vec_part *= -1

    def multiply(self, other: 'RotationQuaterion') -> 'RotationQuaterion':
        """Multiply two rotation quaternions
        Hint: see (10.33)

        As __matmul__ is implemented for this class, you can use:
        q1@q2 which is equivalent to q1.multiply(q2)

        Args:
            other (RotationQuaternion): the other quaternion    
        Returns:
            quaternion_product (RotationQuaternion): the product
        """



        # equation 10.34
        I = np.identity(4)
        S = get_cross_matrix(self.vec_part)
        eps = self.vec_part.reshape(-1,1) #shape (3,1)
        A = np.block([
            [np.array([0]),- eps.T ],
            [eps, S ]
        ])
        q_product = np.dot( self.real_part*I + A, np.append(other.real_part, other.vec_part))
        quaternion_product = RotationQuaterion(q_product[0], q_product[1:])

        # TODO: remove when test is good
        # quaternion_product = solution.quaternion.RotationQuaterion.multiply(self,other)

        return quaternion_product

    def conjugate(self) -> 'RotationQuaterion':
        """Get the conjugate of the RotationQuaternion"""

        # using equation 10.27
        qstar = np.append(self.real_part,-self.vec_part) 

        q_conj = qstar/np.linalg.norm(qstar, ord=2) 
        conj = RotationQuaterion(q_conj[0], q_conj[1:])
        # TODO replace this with your own code
        # conj = solution.quaternion.RotationQuaterion.conjugate(self)

        return conj

    def as_rotmat(self) -> 'ndarray[3,3]':
        """Get the rotation matrix representation of self

        Returns:
            R (ndarray[3,3]): rotation matrix
        """
        # equation 10.38
        phi, theta, psi = self.as_euler()

        # equation 10.18
        R_phi = np.array([
            [1 ,0 ,0 ],
            [0 , np.cos(phi), -np.sin(phi)],
            [0 , np.sin(phi), np.cos(phi)]
        ])
        R_theta = np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0 , 1, 0 ],
            [-np.sin(theta) , 0, np.cos(theta) ]
        ])
        R_psi = np.array([
            [np.cos(psi), - np.sin(psi), 0],
            [np.sin(psi),  np.cos(psi), 0],
            [0, 0, 1]
        ]) 

        R = R_psi@R_theta@R_phi

        # TODO: remove when small numerical error is fixed
        # R = solution.quaternion.RotationQuaterion.as_rotmat(self)
        return R

    @property
    def R(self) -> 'ndarray[3,3]':
        return self.as_rotmat()

    def as_euler(self) -> 'ndarray[3]':
        """Get the euler angle representation of self

        Returns:
            euler (ndarray[3]): extrinsic xyz euler angles (roll, pitch, yaw)
        """

        # equation 10.38
        mu = self.real_part
        eps1, eps2, eps3 = self.vec_part
        phi = np.arctan2(2*(eps3*eps2 + mu*eps1), mu**2 - eps1**2 - eps2**2 + eps3**2)
        theta = np.arcsin( 2*(mu*eps2 - eps1*eps3)) 
        psi = np.arctan2(2*(eps1*eps2 + mu*eps3), mu**2 + eps1**2 - eps2**2 - eps3**2)

        euler = np.array([phi, theta, psi])

        # TODO replace this with your own code
        # euler = solution.quaternion.RotationQuaterion.as_euler(self)

        return euler

    def as_avec(self) -> 'ndarray[3]':
        """Get the angles vector representation of self

        Returns:
            euler (ndarray[3]): extrinsic xyz euler angles (roll, pitch, yaw)
        """
        angle = 2*np.arccos(self.real_part)
        n = self.vec_part/(1- self.real_part**2)**0.5

        # TODO replace this with your own code
        # avec = solution.quaternion.RotationQuaterion.as_avec(self)
        avec = n*angle

        return avec

    @staticmethod
    def from_euler(euler: 'ndarray[3]') -> 'RotationQuaterion':
        """Get a rotation quaternion from euler angles
        usage: rquat = RotationQuaterion.from_euler(euler)

        Args:
            euler (ndarray[3]): extrinsic xyz euler angles (roll, pitch, yaw)

        Returns:
            rquat (RotationQuaternion): the rotation quaternion
        """
        scipy_quat = Rotation.from_euler('xyz', euler).as_quat()
        rquat = RotationQuaterion(scipy_quat[3], scipy_quat[:3])
        return rquat

    def _as_scipy_quat(self):
        """If you're using scipys Rotation class, this can be handy"""
        return np.append(self.vec_part, self.real_part)

    def __iter__(self):
        return iter([self.real_part, self.vec_part])

    def __matmul__(self, other) -> 'RotationQuaterion':
        """Lets u use the @ operator, q1@q2 == q1.multiply(q2)"""
        return self.multiply(other)
