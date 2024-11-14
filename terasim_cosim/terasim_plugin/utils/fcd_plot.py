import matplotlib.pyplot as plt
import time


class DynamicPlot:
    def __init__(self):
        """
        Initializes the dynamic plot with the initial timestamp.
        """
        self.times = []
        self.cav_speeds = []
        self.bv_speeds = []
        self.distances = []

        # Set up the figure and subplots
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(19, 4))

        # Configure the Speed subplot
        self.ax1.set_xlim(0, 8)
        self.ax1.set_ylim(0, 10)  # Fixed 0-10 range for Speed
        self.ax1.set_xlabel("Time (s)", fontsize=24)
        self.ax1.set_ylabel(
            "Speed (m/s)", fontsize=24, labelpad=4
        )  # Added labelpad for spacing
        (self.line1,) = self.ax1.plot(
            [], [], label="AV Speed", color=(1.0, 0.0, 0.0), linewidth=2
        )
        (self.line2,) = self.ax1.plot(
            [], [], label="BV Speed", color=(0, 0.4, 0.8), linewidth=2
        )
        self.ax1.legend(loc="upper left", fontsize=18)
        self.ax1.tick_params(axis="both", which="major", labelsize=18)

        # Configure the Distance subplot
        self.ax2.set_xlim(0, 8)
        self.ax2.set_ylim(0, 25)  # Fixed 0-25 range for Distance
        self.ax2.set_xlabel("Time (s)", fontsize=24)
        self.ax2.set_ylabel(
            "Distance (m)", fontsize=24, labelpad=4
        )  # Added labelpad for spacing
        (self.line3,) = self.ax2.plot(
            [], [], label="Distance", color="green", linewidth=2
        )
        self.ax2.legend(loc="upper left", fontsize=18)
        self.ax2.tick_params(axis="both", which="major", labelsize=18)

        plt.ion()  # Enable interactive mode
        plt.tight_layout(pad=3.0)  # Adjust the layout padding
        plt.show()

    def add_data_point(self, timestamp, cav_speed, bv_speed, distance):
        """
        Adds a new data point to the plot and updates the display.

        Parameters:
        - timestamp (float): The current timestamp.
        - cav_speed (float): Speed value for line 1.
        - bv_speed (float): Speed value for line 2.
        - distance (float): Distance value for the second subplot.
        """
        normalized_time = timestamp
        self.times.append(normalized_time)
        self.cav_speeds.append(cav_speed)
        self.bv_speeds.append(bv_speed)
        self.distances.append(distance)

        # Update data for each line
        self.line1.set_data(self.times, self.cav_speeds)
        self.line2.set_data(self.times, self.bv_speeds)
        self.line3.set_data(self.times, self.distances)

        # Keep a rolling 8-second window on the x-axis
        if normalized_time > 8:
            self.ax1.set_xlim(normalized_time - 8, normalized_time)
            self.ax2.set_xlim(normalized_time - 8, normalized_time)
        else:
            self.ax1.set_xlim(0, 8)
            self.ax2.set_xlim(0, 8)

        # Redraw the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)  # Pause briefly to allow the plot to update


# # Example usage with real-time data updating
# plot = DynamicPlot()

# # Simulate real-time data coming in
# for t in range(0, 100):  # Simulate 20 seconds of data
#     # Generate example data
#     cav_speed = t % 10  # Cyclic speed
#     bv_speed = 10 - t % 10
#     distance = t  # Linear distance increase

#     # Add data to the plot and update in real-time
#     plot.add_data_point(
#         timestamp=t * 0.1, cav_speed=cav_speed, bv_speed=bv_speed, distance=distance
#     )

#     # Simulate a delay to mimic real-time data arrival
#     time.sleep(0.1)

# # End interactive mode and finalize plot display
# plt.ioff()
# plt.show()
