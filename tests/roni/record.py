import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import csv
import math
import matplotlib.pyplot as plt
import numpy as np
import json
import matplotlib.ticker as ticker 
import os
import shutil


class SaveTFNode(Node):
    def __init__(self, model_number):
        super().__init__('save_tf_node')

        self.model_number = model_number

        # Create a TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Open a CSV file to save the transformed data
        self.csv_file = open(f'transformed_to_output{self.model_number}.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header
        self.csv_writer.writerow(['Output_X', 'Output_Y'])

        # Set up a timer to periodically record the transform
        self.timer = self.create_timer(1.0, self.record_transform)  # Record every second

        # Parameters for reverse transformation
        self.origin_x = 18.00
        self.origin_y = 0.16
        self.rotation_degrees = 95
        self.theta = math.radians(self.rotation_degrees)

        # Lists to store points for plotting
        self.transformed_points = []
        self.output_points = []
        self.input_points = []  # Points from JSON file

    def load_input_points(self, json_file):
        try:
            with open(json_file, 'r') as file:
                data = json.load(file)
                self.input_points = [(point[0], point[1]) for point in data]  # Skip first 2 and last 2 points
                self.get_logger().info(f"Loaded {len(self.input_points)} points from {json_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to load points from {json_file}: {e}")

    def load_transformed_points(self, csv_file):
        try:
            with open(csv_file, 'r') as file:
                reader = csv.reader(file)
                next(reader)  # Skip header
                for row in reader:
                    if len(row) < 2 or not row[0] or not row[1]:  # Skip invalid rows
                        self.get_logger().warn(f"Skipping invalid row: {row}")
                        continue
                    try:
                        x, y = float(row[0]), float(row[1])
                        self.output_points.append((x, y))
                        self.get_logger().info(f"Row loaded: X={x}, Y={y}")
                    except ValueError:
                        self.get_logger().error(f"Invalid value in row: {row}")
                self.get_logger().info(f"Loaded {len(self.output_points)} points from {csv_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to load points from {csv_file}: {e}")

    def record_transform(self):
        try:
            # Get the transform from 'map' to 'base_link'
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            # Extract transformed coordinates
            x_transformed = transform.transform.translation.x
            y_transformed = transform.transform.translation.y

            # Reverse transform to original coordinate system
            x_output, y_output = self.reverse_transform(x_transformed, y_transformed)

            # Validate data
            if math.isnan(x_output) or math.isnan(y_output):
                self.get_logger().error("Invalid output coordinates: NaN detected.")
                return

            # Save to CSV
            self.csv_writer.writerow([x_output, y_output])
            self.get_logger().info(f"Saved output coordinates: X={x_output}, Y={y_output}")

            # Store points for plotting
            self.transformed_points.append((x_transformed, y_transformed))
            self.output_points.append((x_output, y_output))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Transform not available yet")

    def reverse_transform(self, x_transformed, y_transformed):
        # Step 1: Translate back
        x_translated = x_transformed - self.origin_x
        y_translated = y_transformed - self.origin_y

        # Step 2: Rotate back
        x_output = x_translated * math.cos(-self.theta) - y_translated * math.sin(-self.theta)
        y_output = x_translated * math.sin(-self.theta) + y_translated * math.cos(-self.theta)

        return x_output, y_output

    def plot_trajectories(self):
        # Plot recorded points, input points, and transformed paths

        plt.figure(figsize=(10, 8))

        # Plot recorded output points as a line
        if self.output_points:
            output_x, output_y = zip(*self.output_points)
            plt.plot(output_x, output_y, color="blue", label="Recorded Path", linestyle="-", marker="x")

        # Plot input points from JSON as a line
        if self.input_points:
            input_x, input_y = zip(*self.input_points)
            plt.plot(input_x, input_y, color="red", label="Input Path", linestyle="-", marker="o")

        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.title(f"Comparison of Recorded and Input Paths - Model {self.model_number}")
        # Plot settings
        plt.axis('equal')  # Ensures equal scaling for both axes
        plt.xlim(min(output_x + input_x) - 0.1, max(output_x + input_x) + 0.1)
        plt.ylim(min(output_y + input_y) - 0.1, max(output_y + input_y) + 0.1)
        ax = plt.gca()
        ax.xaxis.set_major_locator(ticker.MultipleLocator(0.2))
        plt.xticks(rotation=45)  # Rotate labels for readability
        plt.yticks(np.arange(min(output_y + input_y) - 0.1, max(output_y + input_y) + 0.1, 0.1))
        plt.axhline(0, color="black", linewidth=0.5, linestyle="--")
        plt.axvline(0, color="black", linewidth=0.5, linestyle="--")
        plt.legend()
        plt.grid()

        # Save the plot as an image with the model number
        plot_file = f"comparison_of_trajectories{self.model_number}.png"
        plt.savefig(plot_file)
        self.get_logger().info(f"Plot saved as '{plot_file}'")

    def __del__(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info("CSV file closed properly.")

def main(args=None):
    rclpy.init(args=args)

    # Ask for the model number
    valid_model_numbers = {"1f", "1b", "2f", "2b", "3f", "3b", "4f", "4b"}

    model_number = input("Enter the model number (1f, 1b, 2f, 2b, 3f, 3b, 4f, or 4b): ").strip().lower()
    while model_number not in valid_model_numbers:
        model_number = input("Invalid input. Please enter a valid model number (1f, 1b, 2f, 2b, 3f, 3b, 4f, or 4b): ").strip().lower()

  

    # Create the node with the model number
    node = SaveTFNode(model_number)

    # Load input points from JSON file
    json_file = '1b.json'  # Replace with the actual file path
    node.load_input_points(json_file)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Reload transformed points from CSV
        csv_file = f'transformed_to_output{model_number}.csv'
        node.load_transformed_points(csv_file)

        # Plot the trajectories explicitly at the end
        if node.output_points or node.input_points:
            node.plot_trajectories()

        # Define the directory to save the files
        target_directory = "saved_results"
        os.makedirs(target_directory, exist_ok=True)  # Create the directory if it doesn't exist

        # Move files to the target directory
        try:
            shutil.move(f"transformed_to_output{model_number}.csv", os.path.join(target_directory, f"transformed_to_output{model_number}.csv"))
            shutil.move(f"comparison_of_trajectories{model_number}.png", os.path.join(target_directory, f"comparison_of_trajectories{model_number}.png"))
            print(f"Files saved in directory: {target_directory}")
        except Exception as e:
            print(f"Failed to save files: {e}")

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()



if __name__ == '__main__':
    main()