#!/usr/bin/env python3
import os
import csv
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import matplotlib.pyplot as plt


JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']


class JointLogger(Node):

    def __init__(self):
        super().__init__('joint_logger')

        self.desired = {}
        self.actual = {}

        self.start_time = time.monotonic()

        self.time_data = []

        self.desired_data = {joint: [] for joint in JOINTS}
        self.actual_data = {joint: [] for joint in JOINTS}
        self.error_data = {joint: [] for joint in JOINTS}

        self.create_subscription(
            JointState,
            '/desired_joint_states',
            self.desired_callback,
            10
        )

        self.create_subscription(
            JointState,
            '/joint_states',
            self.actual_callback,
            10
        )

        # Record at 10 Hz
        self.create_timer(
            0.1,
            self.record_data
        )

        self.get_logger().info(
            'Joint logger started'
        )


    def desired_callback(self, msg):

        for name, position in zip(msg.name, msg.position):
            self.desired[name] = position


    def actual_callback(self, msg):

        for name, position in zip(msg.name, msg.position):
            self.actual[name] = position


    def record_data(self):

        # Wait until all four joints exist
        if not all(j in self.desired for j in JOINTS):
            return

        if not all(j in self.actual for j in JOINTS):
            return

        t = time.monotonic() - self.start_time

        self.time_data.append(t)

        for joint in JOINTS:

            q_des = self.desired[joint]
            q = self.actual[joint]

            error = q_des - q

            self.desired_data[joint].append(q_des)
            self.actual_data[joint].append(q)
            self.error_data[joint].append(error)


    def save_results(self):

        if len(self.time_data) == 0:
            self.get_logger().warning(
                'No data recorded.'
            )
            return

        results_dir = os.path.expanduser(
            '~/samarth_arm_ws/results'
        )

        os.makedirs(
            results_dir,
            exist_ok=True
        )

        # -----------------------------------------
        # Save CSV
        # -----------------------------------------

        csv_file = os.path.join(
            results_dir,
            'tracking_results.csv'
        )

        with open(csv_file, 'w', newline='') as file:

            writer = csv.writer(file)

            header = ['time']

            for joint in JOINTS:
                header += [
                    f'{joint}_desired',
                    f'{joint}_actual',
                    f'{joint}_error'
                ]

            writer.writerow(header)

            for i, t in enumerate(self.time_data):

                row = [t]

                for joint in JOINTS:

                    row += [
                        self.desired_data[joint][i],
                        self.actual_data[joint][i],
                        self.error_data[joint][i]
                    ]

                writer.writerow(row)


        # -----------------------------------------
        # Calculate RMSE
        # -----------------------------------------

        print('\nTracking RMSE')
        print('-------------')

        for joint in JOINTS:

            errors = self.error_data[joint]

            mse = sum(e * e for e in errors) / len(errors)

            rmse = math.sqrt(mse)

            print(
                f'{joint}: {rmse:.6f} rad'
            )


        # -----------------------------------------
        # Plot desired vs actual
        # -----------------------------------------

        for joint in JOINTS:

            plt.figure()

            plt.plot(
                self.time_data,
                self.desired_data[joint],
                label='Desired'
            )

            plt.plot(
                self.time_data,
                self.actual_data[joint],
                label='Actual'
            )

            plt.xlabel('Time [s]')
            plt.ylabel('Joint Position [rad]')

            plt.title(
                f'{joint}: Desired vs Actual'
            )

            plt.legend()
            plt.grid()

            plot_file = os.path.join(
                results_dir,
                f'{joint}_tracking.png'
            )

            plt.savefig(plot_file)
            plt.close()


        self.get_logger().info(
            f'Results saved to: {results_dir}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = JointLogger()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        print('\nStopping logger...')

    finally:

        node.save_results()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
