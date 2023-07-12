import numpy as np

class OwnShip():
    def __init__(self, dim):
        self.dim = dim
        self.enu_pos = np.empty((0, 2), dtype=float)
        self.shape = np.empty((5, 2), dtype=float)
        self.spd = 0
        self.heading = 0


class TargetShip():
    def __init__(self, dim):
        self.dim = dim
        self.identification_status = False
        self.enu_pos = np.empty((0, 2), dtype=float)
        self.shape = np.empty((5, 2), dtype=float)
        self.spd = 0
        self.heading = 0


def Cal_Shape(Ship):
    heading_rad = np.deg2rad(Ship.heading)
    dim = Ship.dim
    enu_pos = Ship.enu_pos

    temp_x = np.transpose(np.array([2/3, 1/3, -1/3, -1/3, 1/3]))
    temp_x = dim[0] * temp_x
    
    temp_y = np.transpose(np.array([0, 1/2, 1/2, -1/2, -1/2]))
    temp_y = dim[1] * temp_y

    temp = np.hstack((temp_x, temp_y))

    Rotation_mat = np.array([[np.cos(heading_rad), -np.sin(heading_rad)], 
    [np.sin(heading_rad), np.cos(heading_rad)]])

    rt_temp = temp * Rotation_mat + enu_pos[:, np.newaxis]
    shape = np.array(rt_temp)

    return shape

def Cal_VO(OS, TS):
    OS_shape = OS.shape
    TS_shape = TS.shape

    VO = np.empty(float, (3, 2))

    VO[1, :] = OS.enu_pos
    
    bearing = np.arctan(TS_shape[:, 1] - OS.enu_pos[1],
    np.arctan(TS_shape[:, 0]) - OS.enu_pos[0])
    min_idx = np.argmin(bearing)
    max_idx = np.argmax(bearing)

    VO[0, :] = TS_shape[min_idx, :]
    VO[2, :] = TS_shape[max_idx, :]

    VO = VO + enu_pos[:, np.newaxis]

    return VO

def Cal_RV(spd_lim, ang_lim, OS_heading):
    spd_set = np.linspace(spd_lim[0], spd_lim[1], 5)
    ang_set = np.linspace(ang_lim[0], ang_lim[1], 100) + OS_heading
    
    spd_array, ang_array = np.meshgrid(spd_set, ang_set)
    RV = np.vstack((spd_array.flatten(), ang_array.flatten())).T

    return RV

def Cal_RP(RV, OS_enu_pos, tmin):
    RP = Cal_RAP(RV, OS_enu_pos, tmin)
    return RP

def Cal_RAV(RAP, VO):
    if len(RAP) == 0:
        RAV = []
    else:
        polygon_path = mpath.Path(VO)
        test_points = RAP
        results = polygon_path.contains_points(test_points)
        if len(results) == 0:
            RAV = []
        else:
            RAV = np.array(test_points[results])

    return RAV

def Cal_RAP(RAV, OS_enu_pos, tmin):
    if len(RAV) == 0:
        RAP = []
    else:
        RAV_cart = np.zeros_like(RAV) 
        RAV_cart[:, 0] = RAV[:, 0] * np.cos(RAV[:, 1]) # u
        RAV_cart[:, 1] = RAV[:, 0] * np.sin(RAV[:, 1]) # v

        RAP = tmin * RAV_cart + OS_enu_pos[:, np.newaxis]

    return RAP
