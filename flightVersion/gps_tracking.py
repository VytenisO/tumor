import math
import pandas as pd
from os import path, listdir
import matplotlib.pyplot as plt
import numpy as np
import time
import matplotlib.pyplot as pl
from numpy import sin, cos, pi, linspace


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
    altitude = pd.to_numeric(df["alt"]).iloc[-1]

    print(latitude, longitude, altitude)
    return round(latitude, 6), round(longitude, 6), altitude


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

    return distance * 1000


def elevation_angle(height_difference, distance):
    angle_radians = math.atan2(height_difference, distance)  # radians
    angle_degrees = math.degrees(angle_radians)

    return angle_degrees


def deg2rad(deg):
    return deg * pi / 180


def plot_compass_bearing(bearing, elevation, ground_distance, altitude_difference):
    fig = plt.figure(figsize=(12, 8))

    ax = fig.add_subplot(212, polar=True)
    ax.set_yticklabels([])
    ax.set_xticklabels(["E", "", "N", "", "W", "", "S", ""])
    ax.plot([0, np.deg2rad(bearing)], [0, 1])

    # Subplot for the elevation angle
    # ax2 = fig.add_subplot(122)
    # ax2.set_xlim(-1, 1)
    # ax2.set_ylim(-1, 1)
    # ax2.plot([0, 0], [0, math.tan(np.deg2rad(elevation))], label='Elevation angle')
    # ax2.legend()

    ax2 = fig.add_subplot(221)
    ax2.set_xlabel("ground distance (meters)")
    ax2.set_ylabel("altitude difference (meters)")
    ax2.plot(ground_distance, altitude_difference, "o-")

    ax3 = fig.add_subplot(222)
    r = 1.5
    angles = linspace(0 * pi, 2 * pi, 100)
    xs = cos(angles)
    ys = sin(angles)

    ax3.plot(xs, ys, color="green")
    ax3.set_xlim(-r, r)
    ax3.set_ylim(-r, r)
    ax3.set_aspect("equal")
    ax3.set_title(f"Elevation: {round(elevation,2)} deg")
    ax3.plot(r - 0.5, 0, marker="P", color="blue")
    ax3.plot(-r + 0.5, 0, marker="o", color="red")
    ax3.plot([r, -r], [0, 0], color="red")
    ax3.plot([0, r * cos(deg2rad(0))], [0, r * sin(deg2rad(0))], color="red")
    ax3.plot(
        [0, r * cos(deg2rad(elevation))],
        [0, r * sin(deg2rad(elevation))],
        color="black",
    )

    fig.text(
        0.5,
        0,
        f"Bearing: {round(90-bearing, 3)} degrees",
        horizontalalignment="center",
        verticalalignment="bottom",
    )
    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()
    plt.pause(8)
    plt.close()


# print(f"balloon latitude: {balloon_latitude}, longitude: {balloon_longitude}")

# nodeLatitude = 69.29820373738413
# nodeLongitude = 15.997646237074537

operatorLatitude = 69.2957635
operatorLongitude = 16.0310413
operatorAltitude = 13

import folium

while True:
    balloon_latitude, balloon_longitude, balloon_altitude = (
        balloon_longitude_latitude_altitude()
    )
    bearing = calculate_angle(
        operatorLatitude, operatorLongitude, balloon_latitude, balloon_longitude
    )
    lastDistance = distance(
        operatorLatitude, operatorLongitude, balloon_latitude, balloon_longitude
    )
    altitudeDifference = balloon_altitude - operatorAltitude
    print("altitude dif", altitudeDifference)
    elevation = elevation_angle(altitudeDifference, lastDistance)

    # Create a map centered at the average of the coordinates
    map = folium.Map(
        location=[
            (operatorLatitude + balloon_latitude) / 2,
            (operatorLongitude + balloon_longitude) / 2,
        ],
        zoom_start=13,
    )

    # Add markers for the operator and the balloon
    folium.Marker([operatorLatitude, operatorLongitude], popup="Operator").add_to(map)
    folium.Marker([balloon_latitude, balloon_longitude], popup="Balloon").add_to(map)

    # Save the map to an HTML file
    map.save("map.html")
    plot_compass_bearing(bearing, elevation, lastDistance, altitudeDifference)
