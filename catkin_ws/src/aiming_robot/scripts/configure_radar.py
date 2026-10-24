#!/usr/bin/env python3
"""Apply low-noise, demo-range settings to configured simulated FMCW radars."""

import rospy
from dynamic_reconfigure.client import Client


def main():
    rospy.init_node("configure_radar", anonymous=False)
    settings = {
        "ambient_noise": 0,
        "range_max": 12.0,
        "n_cells": 512,
        "n_samples": 3,
        "n_reflections": 2,
    }
    for side in rospy.get_param("~sensor_sides", ["left", "right"]):
        namespace = "/gazebo/radarays/radar_{}_sensor".format(side)
        try:
            client = Client(namespace, timeout=30.0)
            applied = client.update_configuration(settings)
            rospy.loginfo("Configured %s FMCW radar: %s", side, applied)
        except Exception as exc:  # noqa: BLE001 - report unavailable sensor without killing launch
            rospy.logerr("Could not configure %s FMCW radar at %s: %s", side, namespace, exc)


if __name__ == "__main__":
    main()
