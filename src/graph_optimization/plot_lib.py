import numpy as np
import matplotlib.pyplot as plt

def plot_state(truth_x, odom_x, optimized_x, time, file_name):
    """
    Plots the state estimate over time. Then saves the plot figure.

    Args:
        truth_x: The ground truth values of the robot.
        odom_x: The state estimate values of the robot defined by wheel odometry.
        optimized_x: The state estimate values of the robot defined by odom/laser graph optimization.
        time: The time values when the estimated odom/laser values were retrieved.
        file_name: The path and file name that will be written to.
    """
    # Normalize time
    time_error = time[0]
    for i in range(len(time)):
         time[i] = time[i] - time_error

    plt.title("State Estimate Over 1 m")
    plt.ylabel("State")
    plt.xlabel("Time (sec)")

    plt.plot(np.array([time[0], time[-1]]), truth_x, label='ground truth')
    plt.plot(time, odom_x, label='wheel odometry')
    plt.plot(time, optimized_x, label='odom & laser optimized')

    plt.legend()
    plt.savefig(file_name)


def plot_error(truth_x, odom_x, optimized_x, file_name):
    """
    Plots the state error. Then saves the plot figure.

    Args:
        truth_x: The final ground truth value of the robot.
        odom_x: The final state estimate of the robot defined by wheel odometry.
        optimized_x: The final state estimate of the robot defined by odom/laser graph optimization.
        file_name: The path and file name that will be written to.
    """
    readings = ['wheel\n odometry', 'odometry\n& laser\n optimized']
    errors = [odom_x - truth_x, optimized_x - truth_x]

    fig, ax = plt.subplots()
    ax.bar(readings, errors)

    ax.set_title("State Estimate Over 1 m")
    ax.set_ylabel("Errors (m)")

    plt.savefig(file_name)
