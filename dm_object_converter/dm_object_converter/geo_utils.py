# Perception Engine Inc. 2022
import numpy as np

WGS84_F = 298.257223565
JDG2011_F = 298.257222101


def get_lat_long_from_plane(num: int) -> (float, float):
    """Obtains the latitude longitude of the origin of the specified plane

            Args:
              num: Plane number (1 to 19)

            Returns:
              Latitude and Longitude

            Raises:
              Nothing
        """
    lon_deg = lat_deg = 0
    if num == 1:
        lon_deg = 33
        lat_deg = 129 + 30 / 60
    elif num == 2:
        lon_deg = 33
        lat_deg = 131
    elif num == 3:
        lon_deg = 36
        lat_deg = 132
    elif num == 4:
        lon_deg = 33
        lat_deg = 133 + 30 / 60
    elif num == 5:
        lon_deg = 36
        lat_deg = 134 + 20 / 60
    elif num == 6:
        lon_deg = 36
        lat_deg = 136
    elif num == 7:
        lon_deg = 36
        lat_deg = 137 + 10 / 60
    elif num == 8:
        lon_deg = 36
        lat_deg = 138 + 30 / 60
    elif num == 9:
        lon_deg = 36
        lat_deg = 139 + 50 / 60
    elif num == 10:
        lon_deg = 40
        lat_deg = 140 + 50 / 60
    elif num == 11:
        lon_deg = 44
        lat_deg = 140 + 15 / 60
    elif num == 12:
        lon_deg = 44
        lat_deg = 142 + 15 / 60
    elif num == 13:
        lon_deg = 44
        lat_deg = 144 + 15 / 60
    elif num == 14:
        lon_deg = 26
        lat_deg = 142
    elif num == 15:
        lon_deg = 26
        lat_deg = 127 + 30 / 60
    elif num == 16:
        lon_deg = 26
        lat_deg = 124
    elif num == 17:
        lon_deg = 26
        lat_deg = 131
    elif num == 18:
        lon_deg = 20
        lat_deg = 136
    elif num == 19:
        lon_deg = 26
        lat_deg = 154
    return lat_deg, lon_deg


def xyp_to_lat_lon(x: float, y: float, plane_num: int, flattening_value=WGS84_F) -> (float, float):
    """Converts a point in Japan Plane Rectangular Coordinate System to Latitude Longitude

        Args:
          x: X coordinate inside the specified plane number
          y: Y coordinate inside the specified plane number
          plane_num: Plane number (1 to 19)
          flattening_value: Earth Flattening value

        Returns:
          Latitude and Longitude

        Raises:
          Nothing
    """
    lambda0_deg, phi0_deg = get_lat_long_from_plane(plane_num)
    # 平面直角座標系原点をラジアンに直す
    phi0_rad = np.deg2rad(phi0_deg)
    lambda0_rad = np.deg2rad(lambda0_deg)

    # 補助関数
    def A_array(n):
        A0 = 1 + (n ** 2) / 4. + (n ** 4) / 64.
        A1 = -     (3. / 2) * (n - (n ** 3) / 8. - (n ** 5) / 64.)
        A2 = (15. / 16) * (n ** 2 - (n ** 4) / 4.)
        A3 = -   (35. / 48) * (n ** 3 - (5. / 16) * (n ** 5))
        A4 = (315. / 512) * (n ** 4)
        A5 = -(693. / 1280) * (n ** 5)
        return np.array([A0, A1, A2, A3, A4, A5])

    def beta_array(n):
        b0 = np.nan  # dummy
        b1 = (1. / 2) * n - (2. / 3) * (n ** 2) + (37. / 96) * (n ** 3) - (1. / 360) * (n ** 4) - (81. / 512) * (n ** 5)
        b2 = (1. / 48) * (n ** 2) + (1. / 15) * (n ** 3) - (437. / 1440) * (n ** 4) + (46. / 105) * (n ** 5)
        b3 = (17. / 480) * (n ** 3) - (37. / 840) * (n ** 4) - (209. / 4480) * (n ** 5)
        b4 = (4397. / 161280) * (n ** 4) - (11. / 504) * (n ** 5)
        b5 = (4583. / 161280) * (n ** 5)
        return np.array([b0, b1, b2, b3, b4, b5])

    def delta_array(n):
        d0 = np.nan  # dummy
        d1 = 2. * n - (2. / 3) * (n ** 2) - 2. * (n ** 3) + (116. / 45) * (n ** 4) + (26. / 45) * (n ** 5) - (
                    2854. / 675) * (n ** 6)
        d2 = (7. / 3) * (n ** 2) - (8. / 5) * (n ** 3) - (227. / 45) * (n ** 4) + (2704. / 315) * (n ** 5) + (
                    2323. / 945) * (n ** 6)
        d3 = (56. / 15) * (n ** 3) - (136. / 35) * (n ** 4) - (1262. / 105) * (n ** 5) + (73814. / 2835) * (n ** 6)
        d4 = (4279. / 630) * (n ** 4) - (332. / 35) * (n ** 5) - (399572. / 14175) * (n ** 6)
        d5 = (4174. / 315) * (n ** 5) - (144838. / 6237) * (n ** 6)
        d6 = (601676. / 22275) * (n ** 6)
        return np.array([d0, d1, d2, d3, d4, d5, d6])

    # 定数 (a, F: 世界測地系-測地基準系1980（GRS80）楕円体)
    m0 = 0.9999
    a = 6378137.
    F = flattening_value

    # (1) n, A_i, beta_i, delta_iの計算
    n = 1. / (2 * F - 1)
    A_array = A_array(n)
    beta_array = beta_array(n)
    delta_array = delta_array(n)

    # (2), S, Aの計算
    A_ = ((m0 * a) / (1. + n)) * A_array[0]
    S_ = ((m0 * a) / (1. + n)) * (A_array[0] * phi0_rad + np.dot(A_array[1:], np.sin(2 * phi0_rad * np.arange(1, 6))))

    # (3) xi, etaの計算
    xi = (x + S_) / A_
    eta = y / A_

    # (4) xi', eta'の計算
    xi2 = xi - np.sum(np.multiply(beta_array[1:],
                                  np.multiply(np.sin(2 * xi * np.arange(1, 6)),
                                              np.cosh(2 * eta * np.arange(1, 6)))))
    eta2 = eta - np.sum(np.multiply(beta_array[1:],
                                    np.multiply(np.cos(2 * xi * np.arange(1, 6)),
                                                np.sinh(2 * eta * np.arange(1, 6)))))

    # (5) chiの計算
    chi = np.arcsin(np.sin(xi2) / np.cosh(eta2))  # [rad]
    latitude = chi + np.dot(delta_array[1:], np.sin(2 * chi * np.arange(1, 7)))  # [rad]

    # (6) 緯度(latitude), 経度(longitude)の計算
    longitude = lambda0_rad + np.arctan(np.sinh(eta2) / np.cos(xi2))  # [rad]

    # ラジアンを度になおしてreturn
    return np.rad2deg(latitude), np.rad2deg(longitude)  # [deg]


def lat_lon_to_xyp(lat: float, long: float, plane_num: int, flattening_value=WGS84_F) -> (float, float):
    """Converts a point in Latitude Longitude to Japan Plane Rectangular Coordinate System

            Args:
              lat: latitude in decimal
              long: longitude in decimal
              plane_num: Plane number (1 to 19)
              flattening_value: Earth flattening value

            Returns:
              X,Y inside the specified plane number

            Raises:
              Nothing
    """
    lambda0_deg, phi0_deg = get_lat_long_from_plane(plane_num)
    # 緯度経度・平面直角座標系原点をラジアンに直す
    phi_rad = np.deg2rad(lat)
    lambda_rad = np.deg2rad(long)
    phi0_rad = np.deg2rad(phi0_deg)
    lambda0_rad = np.deg2rad(lambda0_deg)

    # 補助関数
    def A_array(n):
        A0 = 1 + (n ** 2) / 4. + (n ** 4) / 64.
        A1 = -     (3. / 2) * (n - (n ** 3) / 8. - (n ** 5) / 64.)
        A2 = (15. / 16) * (n ** 2 - (n ** 4) / 4.)
        A3 = -   (35. / 48) * (n ** 3 - (5. / 16) * (n ** 5))
        A4 = (315. / 512) * (n ** 4)
        A5 = -(693. / 1280) * (n ** 5)
        return np.array([A0, A1, A2, A3, A4, A5])

    def alpha_array(n):
        a0 = np.nan  # dummy
        a1 = (1. / 2) * n - (2. / 3) * (n ** 2) + (5. / 16) * (n ** 3) + (41. / 180) * (n ** 4) - (127. / 288) * (
                    n ** 5)
        a2 = (13. / 48) * (n ** 2) - (3. / 5) * (n ** 3) + (557. / 1440) * (n ** 4) + (281. / 630) * (n ** 5)
        a3 = (61. / 240) * (n ** 3) - (103. / 140) * (n ** 4) + (15061. / 26880) * (n ** 5)
        a4 = (49561. / 161280) * (n ** 4) - (179. / 168) * (n ** 5)
        a5 = (34729. / 80640) * (n ** 5)
        return np.array([a0, a1, a2, a3, a4, a5])

    # 定数 (a, F: 世界測地系-測地基準系1980（GRS80）楕円体)
    m0 = 0.9999
    a = 6378137.
    F = flattening_value

    # (1) n, A_i, alpha_iの計算
    n = 1. / (2 * F - 1)
    A_array = A_array(n)
    alpha_array = alpha_array(n)

    # (2), S, Aの計算
    A_ = ((m0 * a) / (1. + n)) * A_array[0]  # [m]
    S_ = ((m0 * a) / (1. + n)) * (
                A_array[0] * phi0_rad + np.dot(A_array[1:], np.sin(2 * phi0_rad * np.arange(1, 6))))  # [m]

    # (3) lambda_c, lambda_sの計算
    lambda_c = np.cos(lambda_rad - lambda0_rad)
    lambda_s = np.sin(lambda_rad - lambda0_rad)

    # (4) t, t_の計算
    t = np.sinh(np.arctanh(np.sin(phi_rad)) - ((2 * np.sqrt(n)) / (1 + n)) * np.arctanh(
        ((2 * np.sqrt(n)) / (1 + n)) * np.sin(phi_rad)))
    t_ = np.sqrt(1 + t * t)

    # (5) xi', eta'の計算
    xi2 = np.arctan(t / lambda_c)  # [rad]
    eta2 = np.arctanh(lambda_s / t_)

    # (6) x, yの計算
    x = A_ * (xi2 + np.sum(np.multiply(alpha_array[1:],
                                       np.multiply(np.sin(2 * xi2 * np.arange(1, 6)),
                                                   np.cosh(2 * eta2 * np.arange(1, 6)))))) - S_  # [m]
    y = A_ * (eta2 + np.sum(np.multiply(alpha_array[1:],
                                        np.multiply(np.cos(2 * xi2 * np.arange(1, 6)),
                                                    np.sinh(2 * eta2 * np.arange(1, 6))))))  # [m]
    # return
    return x, y  # [m]
