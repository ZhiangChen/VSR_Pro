import csv
from typing import List

class DataLogger:
    """
    A class to log simulation data such as joint positions, velocities, and external object states.

    Attributes:
        filename (str): Name of the file where data will be logged.
    """

    def __init__(self, filename: str):
        """
        Initializes the DataLogger with a filename.

        Args:
            filename (str): Name of the file where data will be logged.
        """
        self.filename = filename

    def log_data(self, timestamps: List[float], joint_positions: List[List[float]], joint_velocities: List[List[float]], object_positions: List[List[float]], object_orientations: List[List[float]]) -> None:
        """
        Logs simulation data to a CSV file.

        Args:
            timestamps (list[float]): List of time values.
            joint_positions (list[list[float]]): Joint positions at each time step.
            joint_velocities (list[list[float]]): Joint velocities at each time step.
            object_positions (list[list[float]]): External object positions (e.g., rock on pedestal).
            object_orientations (list[list[float]]): External object orientations (e.g., rock's orientation).
        """
        with open(self.filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "Joint Positions", "Joint Velocities", "Object Position", "Object Orientation"])
            for i in range(len(timestamps)):
                writer.writerow([
                    timestamps[i],
                    joint_positions[i],
                    joint_velocities[i],
                    object_positions[i],
                    object_orientations[i]
                ])

    def log_error(self, error_message: str) -> None:
        """
        Logs an error message to the log file.

        Args:
            error_message (str): Error message to be logged.
        """
        with open(self.filename, 'a') as file:
            file.write(f"ERROR: {error_message}\n")
