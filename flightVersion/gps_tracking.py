import math
import pandas as pd
from os import path, listdir
import matplotlib.pyplot as plt
import numpy as np


def calculate_angle(lat1, lon1, lat2, lon2):
    # degrees to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    diff_lon = lon2 - lon1
    diff_lat = lat2 - lat1

    # Calculate the angle
    x = -math.sin(diff_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (
        math.sin(lat1) * math.cos(lat2) * math.cos(diff_lon)
    )

    initial_bearing = math.atan2(x, y)

    bearing_deg = math.degrees(initial_bearing)
    print("initial bearing: ", bearing_deg)
    compass_bearing = (bearing_deg + 90) % 360
    print("compass bearing: ", compass_bearing)

    return compass_bearing


file_times = [
    (f, path.getmtime(f)) for f in listdir(".") if path.isfile(f) and f[-3:] == "csv"
]
sorted_files = sorted(file_times, key=lambda x: x[1], reverse=True)
data_source = sorted_files[0][0]


def balloon_longitude_latitude_altitude():
    df = pd.read_csv(data_source, comment="#")
    latitude = (
        pd.to_numeric(df["lat_deg"])
        + pd.to_numeric(df["lat_min"]) / 60
        + pd.to_numeric(df["lat_sec"]) / 3600
    ).iloc[-1]
    longitude = (
        pd.to_numeric(df["lon_deg"])
        + pd.to_numeric(df["lon_min"]) / 60
        + pd.to_numeric(df["lon_sec"]) / 3600
    ).iloc[-1]
    altitude = pd.to_numeric(df["AL_1"]).iloc[-1]

    print(latitude, longitude, altitude)
    return round(latitude, 6), round(longitude, 6), altitude


balloon_latitude, balloon_longitude, balloon_altitude = balloon_longitude_latitude_altitude()
print(f"balloon latitude: {balloon_latitude}, longitude: {balloon_longitude}")

nodeLatitude = 69.29820373738413
nodeLongitude = 15.997646237074537

latitude = 69.2958424
longitude = 16.0309346


def distance(lat1, lon1, lat2, lon2):  # returns distance in meters
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    diff_lon = lon2 - lon1
    diff_lat = lat2 - lat1

    a = (
        math.sin(diff_lat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(diff_lon / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = 6371 * c  # Earth radius * angle between the two points

    return distance


def elevation_angle(height_difference, distance):
    angle_radians = math.atan2(height_difference, distance)  # radians
    angle_degrees = math.degrees(angle_radians)

    return angle_degrees


def plot_compass_bearing(bearing, elevation):
    fig = plt.figure()
    ax = fig.add_subplot(111, polar=True)

    ax.set_yticklabels([])
    ax.set_xticklabels(["E", "", "N", "", "W", "", "S", ""])

    ax.plot([0, np.deg2rad(bearing)], [0, 1])

    ax.text(
        0.0,
        1.05,
        f"Bearing: {90-bearing} degrees\nElevation: {elevation} degrees",
        horizontalalignment="center",
        verticalalignment="center",
        transform=ax.transAxes,
    )

    plt.show()


bearing = calculate_angle(latitude, longitude, nodeLatitude, nodeLongitude)


elevation = elevation_angle(
    balloon_altitude/1000, distance(latitude, longitude, nodeLatitude, nodeLongitude)
)

plot_compass_bearing(bearing, elevation)