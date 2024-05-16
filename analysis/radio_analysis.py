import math
import pandas as pd
from os import path, listdir
import matplotlib.pyplot as plt
import numpy as np


def load_data():
    df = pd.read_csv(
        "flightVersion/flight_records/GS_Andenes/Flight_data_GS_Andenes_complete.csv"
    )
    df2 = pd.read_csv(
        "flightVersion/flight_records/GS_Andenes/TUMOR_Altitude_corrected.csv"
    )
    merged = pd.merge(df, df2, on="time_GPS", how="inner")
    return merged


df = load_data()
if "Altitude_Corrected (m)" in df.columns:
    print("'Altitude_Corrected (m)' is in df")
else:
    print("'Altitude_Corrected (m)' is not in df")
# starte motta på altitude 20 meter og sluttet på 900, mottok hele veien opp til 22650 meter
# 11:24 - 13:27 flighttime, 124 minutter
# for pakketap trengs antall totale pakker forventet, finner dette ved vite hvor lenge vi opererte og dele på 2.1 sekunder.

received_packages = len(df)
expected_packages = 60 * 124 / 2.1
rate = received_packages / expected_packages
packets_lost = 1 - rate

# plot hvor pakkene forsvinnner ifht. altitude, kan se om det er en sammenheng mellom høyde og pakketap
# plot hvor mange pakker som forsvinner ifht. tid, kan se om det er en sammenheng mellom tid og pakketap, fjern outliers
# plot pakketap ifht distance, kan se om det er en sammenheng mellom distance og pakketap


# hvor mange av pakkene som har dårlig data vet vi ikke, da må vi vite når sensorene gir dårlig data.
# linje 2330 8 minutter diff, men bare 2 meter forskjell, kan være dårlig data.
# timestamps = pd.to_datetime(df["time_GPS"])
def packet_loss(df):
    df["missing_packets"] = 0

    df["time_gps_diff"] = df["time_gps"].diff()
    df = df[
        df["time_gps_diff"] <= 600
    ]  # remove rows when the difference is more than 10 minutes

    df = df[df["time_gps_diff"] > 0]  # row should be after the previous
    lost_packet_mask = df["time_gps_diff"] > 3.5  # packages are sent every 2.1 seconds
    # Set 'altitude' for lost packet rows to be the average of the prior and curent
    df.loc[lost_packet_mask, "Altitude_Corrected (m)"] = (
        df["Altitude_Corrected (m)"].shift(-1) + df["Altitude_Corrected (m)"]
    ) / 2

    df_missing_packets = df.loc[lost_packet_mask].copy()
    df_missing_packets["missing_packets"] = df_missing_packets["time_gps_diff"] / 2.1
    rolling_mean_lost_packets = (
        df_missing_packets["missing_packets"].rolling(window=5, min_periods=0).mean()
    )
    print(df_missing_packets)
    plt.figure(figsize=(10, 6))
    plt.scatter(
        df_missing_packets["time_gps_diff"],
        df_missing_packets["Altitude_Corrected (m)"],
        marker="o",
        s=df_missing_packets["missing_packets"] * 1.2,
        color="blue",
        label="Missing Packets",
    )
    plt.plot(
        rolling_mean_lost_packets,
        df_missing_packets["Altitude_Corrected (m)"],
        color="red",
        label="Rolling mean, missing packets",
    )
    plt.xlabel("radio silence (seconds), packet lost every 2 seconds")
    plt.ylabel("Altitude (meters)")
    plt.title("Altitude vs radio silence")
    plt.grid(True)
    plt.legend()
    plt.show()
    plt.clf()


def uv(df):
    UV0 = pd.to_numeric(df["UV_0"])
    UV1 = pd.to_numeric(df["UV_1"])
    UV2 = pd.to_numeric(df["UV_2"])
    UV3 = pd.to_numeric(df["UV_3"])

    return UV0, UV1, UV2, UV3


def uv_running_average(UV0, UV1, UV2, UV3, window=5):
    UV0_avg = UV0.rolling(window=5, min_periods=0).mean()
    UV1_avg = UV1.rolling(window=5, min_periods=0).mean()
    UV2_avg = UV2.rolling(window=5, min_periods=0).mean()
    UV3_avg = UV3.rolling(window=5, min_periods=0).mean()

    return {
        "UV0_avg": UV0_avg,
        "UV1_avg": UV1_avg,
        "UV2_avg": UV2_avg,
        "UV3_avg": UV3_avg,
        "total_average": (UV1_avg + UV2_avg + UV3_avg + UV0) / 4,
    }


def uv_running_median(UV, window=5, step=300):
    UV_avg = UV.rolling(window=5, min_periods=0).median()

    return {
        "UV_avg": UV_avg,
    }


def sort_uv_by_altitude(UV0, UV1, UV2, UV3, altitudes):
    df = pd.DataFrame(
        {"UV0": UV0, "UV1": UV1, "UV2": UV2, "UV3": UV3, "Altitude": altitudes}
    )

    sorted_df = df.sort_values(by="Altitude")
    return sorted_df


def remove_high_uv_values(df):
    mask0 = df["UV_0"] <= 8000
    mask1 = df["UV_1"] <= 8000
    mask2 = df["UV_2"] <= 8000
    mask3 = df["UV_3"] <= 8000

    mask = mask0 & mask1 & mask2 & mask3
    df = df[mask]
    return df


# will return the max values across the four sensors.
def max_uv_sensor(df):
    uv_columns = ["UV_0", "UV_1", "UV_2", "UV_3"]
    max_uv = df[uv_columns].max(axis=1)
    # print(max_uv)
    return pd.DataFrame({"max": max_uv})


cleanedDf = df[df["Altitude_est"] <= 25000]
cleanedDf = cleanedDf[cleanedDf["Altitude_est"] >= 19]
cleanedDf["time_gps"] = pd.to_timedelta(cleanedDf["time_GPS"]).dt.total_seconds()
cleanedDf = cleanedDf[
    (cleanedDf["time_gps"] >= 41000) & (cleanedDf["time_gps"] <= 48500)
]
packet_loss(cleanedDf)

cleanedDf = remove_high_uv_values(cleanedDf)
sorted_df = sort_uv_by_altitude(
    *uv(cleanedDf),
    pd.to_numeric(cleanedDf["Altitude_est"]),
)
sunFacers = max_uv_sensor(cleanedDf)
# exit()


sunFacingAverage = uv_running_median(sunFacers, window=5, step=None)
plt.plot(
    sunFacingAverage["UV_avg"][::300],
    cleanedDf["Altitude_est"][::300],
    label="sun facing median",
    color="blue",
)
plt.scatter(
    sunFacingAverage["UV_avg"],
    cleanedDf["Altitude_est"],
    label="sun facing median",
    s=1,
    color="red",
)
plt.xlabel("Sun facing median")
plt.ylabel("Altitude")
plt.legend()
plt.show()
plt.clf()

exit()
sunFacingAverage_derivative = sunFacingAverage["UV_avg"].diff()
plt.scatter(
    sunFacingAverage_derivative,
    cleanedDf["Altitude"],
    label="Derivative of sun facing average",
    s=1,
)
plt.xlabel("derivative")
plt.ylabel("Altitude")
plt.title("Derivative of Sun Facing Average")
plt.legend()
plt.show()

uv2 = uv_running_average(
    cleanedDf["UV1"], cleanedDf["UV2"], cleanedDf["UV3"], cleanedDf["UV0"], window=5
)["UV2_avg"]

plt.scatter(cleanedDf["Altitude"], uv2, label="UV2", s=10)
plt.xlabel("Altitude")
plt.ylabel("UV2")
plt.legend()
plt.show()
plt.pause(1)

UVtotalAverage = uv_running_average(
    cleanedDf["UV0"], cleanedDf["UV1"], cleanedDf["UV2"], cleanedDf["UV3"], window=5
)["total_average"]

plt.scatter(cleanedDf["Altitude"], UVtotalAverage, label="total_average", s=10)
plt.xlabel("Altitude")
plt.ylabel("total average")
plt.legend()
plt.show()

altitudess = altitudes()
topIdx = altitudess.idxmax()
print(f"largest alti: {altitudess[topIdx]}")

# UV1_ascent = uv1[:topIdx]
# UV2_ascent = uv2[:topIdx]
# UV3_ascent = uv3[:topIdx]

# UV1_descent = uv1[topIdx::-1]
# UV2_descent = uv2[topIdx::-1]
# UV3_descenst = uv3[topIdx::-1]

# boom = uv_running_average(UV1_ascent,UV2_ascent, UV3_ascent)["UV2_avg"]
# print(uv_running_average(UV1_ascent,UV2_ascent, UV3_ascent)["UV2_avg"])
# print(uv_running_average(UV1_descent, UV2_descent, UV3_descenst)["UV2_avg"])
