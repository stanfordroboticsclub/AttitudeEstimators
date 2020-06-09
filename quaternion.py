import numpy as np

class Quaternion:

    @classmethod
    def Identity(cls):
        return Quaternion( [1, 0, 0, 0] )

    @classmethod
    def fromGyro(cls, w, dt):
        angle = np.linalg.norm(w) * dt
        return Quaternion.fromAxisAngle( w, angle )

    @classmethod
    def fromNudge(cls, nudge):
        w = np.sqrt(1 - np.linalg.norm(nudge)**2)
        return Quaternion( [w, nudge[0], nudge[1], nudge[2]] )

    @classmethod
    def fromAxisAngle(cls, axis, angle):
        if(angle == 0):
            return Quaternion.Identity()
        axis = axis/np.linalg.norm(axis)
        c = np.cos(angle/2)
        s = np.sin(angle/2)
        return Quaternion( [c, s * axis[0], s * axis[1], s * axis[2]] )

    @classmethod
    def fromEuler(cls, yaw, pitch, roll):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return Quaternion([w,x,y,z])
    
    def toEuler(self):
        w, x, y, z = self.q

        # roll (x-axis)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis)
        sinp = 2 * (w * y - z * x)
        if (abs(sinp) >= 1):
            # use 90 degrees if out of range
            pitch = np.sign(sinp)*(np.pi/2)
        else:
            pitch = np.arcsin(sinp);

        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return (yaw, pitch, roll)

    def __init__(self, array):
        self.q = np.array(array)

    def __matmul__(self, other):
        w0, x0, y0, z0 = other.q
        w1, x1, y1, z1 = self.q
        return Quaternion([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                         x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                         -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                         x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0])

    @property
    def T(self):
        w, x, y, z = self.q
        return Quaternion([w, -x, -y, -z])

    def get_matrix(self):
        w = self.q[0]
        v = self.q[1:]

        return (w**2 - np.linalg.norm(v)**2) * np.eye(3) \
                    + 2 * np.outer(v, v) \
                    + 2 * w * self.skew_matrix(v)

    @staticmethod
    def skew_matrix(v):
        # also the cross product matrix
        x, y, z = v
        return np.array ([[ 0,-z, y],
                          [ z, 0,-x],
                          [-y, z, 0]])

    def normalize(self):
        return Quaternion( self.q / np.linalg.norm(self.q) )

    def rotate(self, vector):
        assert len(vector) == 3
        tmp = Quaternion([0, vector[0], vector[1], vector[2]])
        out = self @ tmp @ self.T
        return out.q[1:]
