from math_helpers import *
import pyproj
from scipy.spatial.transform import Rotation
from geodesy.utm import gridZone
import string


def grid_convergence(lat, lon, radians=False):
    """
    Given the latitude and longitude of a position, calculate the grid convergence
    Args:
        lat: latitude (degrees or radians)
        lon: longitude (degrees or radians)
        radians: true if lat/lon in radians

    Returns: gamma, the grid convergence angle in radians or degrees

    """
    lon0, lat0, _ = utm_origin_lla(lat, lon, radians=radians)
    if radians:
        return atan(tan(lon - lon0)*sin(lat))
    else:
        return rad2deg(atan(tand(lon - lon0)*sind(lat)))


def lla_to_ecef(lat, lon, alt=0.0, radians=False):
    """
    Convert Latitude, Longitude and altitude into ECEF coordinates
    Args:
        lat: latitude (degrees or radians)
        lon: longitude (degrees or radians)
        alt: altitude (metres)
        radians: true if lat/lon in radians

    Returns:
        x, y, z coordinates in ECEF (m)

    """
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    return pyproj.transform(lla, ecef, lon, lat, alt, radians=radians)

def ecef_to_lla(x, y, z, radians=False):
    """
    Convert ECEF x, y and z into longitude, latitude, altitude
    Args:
        x: ECEF X (m)
        y: ECEF Y (m)
        z: ECEF Z (m)
        radians: true if lat/lon in radians

    Returns:
        longitude, latitude, altitude

    """
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    return pyproj.transform(ecef, lla, x, y, z, radians=radians)


def latlong_ecef_enu_rotation(lat, lon, radians=False):
    """
    Determine rotation from ECEF frame to ENU frame given latitude and longitude

    Args:
        lat: latitude (degrees or radians)
        lon: longitude (degrees or radians)
        radians: true if lat/lon in radians

    Returns: Quaternion rotation from ECEF to ENU

    """
    if not radians:
        return Rotation.from_dcm([[-sind(lon), cosd(lon), 0],
                                  [-cosd(lon) * sind(lat), -sind(lon) * sind(lat), cosd(lat)],
                                  [cosd(lon) * cosd(lat), sind(lon) * cosd(lat), sind(lat)]]).as_quat().tolist()
    else:
        return Rotation.from_dcm([[-sin(lon), cos(lon), 0],
                                  [-cos(lon) * sin(lat), -sin(lon) * sin(lat), cos(lat)],
                                  [cos(lon) * cos(lat), sin(lon) * cos(lat), sin(lat)]]).as_quat().tolist()


def is_south(band):
    """
    Return if band is in southern hemisphere
    """
    alpha = {c: num for num, c in enumerate(string.ascii_uppercase)}
    return alpha[band.upper()] < 12


def lla_to_utm(lat, lon, alt=0.0, radians=False):
    """
    Given Latitude Longitude, Altitude, return the UTM position
    Args:
        lat: latitude (degrees or radians)
        lon: longitude (degrees or radians)
        alt: altitude (m)
        radians: true if lat/lon in radians

    Returns: Eastings (m), Northings (m), Altitude (m), Zone, Band

    """
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    zone, band = gridZone(lat, lon)
    utm = pyproj.Proj(proj='utm', ellps='WGS84', datum='WGS84', zone=zone, south=is_south(band))
    return pyproj.transform(lla, utm, lon, lat, alt, radians=radians) + (zone, band)


def ecef_to_utm(x, y, z):
    """
    Given ECEF coordinates of a position, return the UTM position and rotation from ECEF frame to ENU frame
    Args:
        x: ECEF X coordinate (m)
        y: ECEF Y coordinate (m)
        z: ECEF Z coordinate (m)

    Returns:
        eastings, northings, altitude, quaternion ECEF to ENU (qx, qy, qz, qw)

    """
    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    lon, lat, alt = pyproj.transform(ecef, lla, x, y, z, radians=False)
    zone, band = gridZone(lat, lon)
    utm = pyproj.Proj(proj='utm', ellps='WGS84', datum='WGS84', zone=zone, south=is_south(band))
    utmpos = pyproj.transform(ecef, utm, x, y, z)
    return utmpos + tuple(latlong_ecef_enu_rotation(lat, lon))

def utm_origin_lla(lat, lon, radians=False):
    """
    Given latitude and longitude, return the origin of the UTM frame in latitude and longitude
    Args:
        lat: latitude (degrees or radians)
        lon: longitude (degrees or radians)
        radians: true if lat/lon in radians

    Returns:
        utm origin: (longitude, latitude, altitude)

    """
    origin = [500000.0, 0.0, 0.0]
    zone, band = gridZone(lat, lon)
    utm = pyproj.Proj(proj='utm', ellps='WGS84', datum='WGS84', zone=zone, south=is_south(band))
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    return pyproj.transform(utm, lla, *origin, radians=radians)


def utm_origin_ecef(x, y, z):
    """
    Given ECEF coordinates, return origin of UTM zone in ECEF coordinates
    Args:
        x: ECEF X coordinate (m)
        y: ECEF Y coordinate (m)
        z: ECEF Z coordinate (m)

    Returns:
        UTM origin in ECEF (X, Y, Z), metres

    """
    lon, lat, alt = ecef_to_lla(x, y, z)
    lon, lat, alt = utm_origin_lla(lat, lon)
    return lla_to_ecef(lat, lon, alt)


if __name__=="__main__":
    # Reference lat, lon, alt ground truths
    latitude, longitude, altitude = [55.605312, 12.799501, 0.0]
    # ECEF ground truths
    XT, YT, ZT = 3521455, 800023, 5239744
    # UTM ground truths
    EastT, NorthT, AltT = 361377, 6164350, 0.0
    # ECEF calculation
    X, Y, Z = lla_to_ecef(latitude, longitude, altitude, radians=False)
    # UTM and Rotation Calculation
    East, North, Alt, QX, QY, QZ, QW = ecef_to_utm(X, Y, Z)
    # UTM origin calculation
    lonO, latO, altO = utm_origin_lla(latitude, longitude)
    XO, YO, ZO = utm_origin_ecef(X, Y, Z)

    latitude1, longitude1, altitude1 = [55.605312, 12.8, 0.0]
    X1, Y1, Z1 = lla_to_ecef(latitude1, longitude1, altitude1, radians=False)
    East1, North1, Alt1, QX1, QY1, QZ1, QW1 = ecef_to_utm(X1, Y1, Z1)
    dist = sqrt((Z1-Z)**2+(X1-X)**2+(Y1-Y)**2)
    utmdist = sqrt((East1-East)**2+(North1-North)**2+(Alt1-Alt)**2)


