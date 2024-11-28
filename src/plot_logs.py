import csv
import matplotlib.pyplot as plt

def read_execution_times(file_path):
    """
    Reads execution times from a CSV file with a header 'execution_time'.

    Args:
        file_path (str): The path to the CSV file.

    Returns:
        list: A list of execution times as floats.
    """
    execution_times = []
    with open(file_path, mode='r') as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            if 'execution_time' in row:
                execution_times.append(float(row['execution_time']))
    return execution_times

def plot_execution_times(parallel_times, serial_times, output_file):
    """
    Plots execution times for parallel and serial execution and saves the plot.

    Args:
        parallel_times (list): Execution times for the parallel implementation.
        serial_times (list): Execution times for the serial implementation.
        output_file (str): The file name to save the plot image.
    """
    plt.figure(figsize=(10, 6))
    plt.plot(parallel_times, label="Parallel Execution", marker='o')
    plt.plot(serial_times, label="Serial Execution", marker='s')

    plt.xlabel("Data Points")
    plt.ylabel("Execution Time (s)")
    plt.title("Comparison of Parallel vs Serial Execution Times")
    plt.legend()
    plt.grid()

    # Save the plot as an image file
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.show()

# File paths
parallel_file = 'parallel.csv'
serial_file = 'serial.csv'
output_file = '../img/execution_times_comparison.png'

# Read execution times
parallel_times = read_execution_times(parallel_file)
serial_times = read_execution_times(serial_file)

# Ensure both lists are the same length for proper plotting
if len(parallel_times) != len(serial_times):
    print("Error: The number of execution times in both files do not match.")
else:
    # Plot execution times and save the plot
    plot_execution_times(parallel_times, serial_times, output_file)
    print(f"Plot saved as '{output_file}'.")



# Calculate the average execution times
average_parallel_time = sum(parallel_times) / len(parallel_times)
average_serial_time = sum(serial_times) / len(serial_times)

print(f"Average Parallel Execution Time: {average_parallel_time:.2f} seconds")
print(f"Average Serial Execution Time: {average_serial_time:.2f} seconds")


# Calculate the speedup factor
speedup_factor = average_serial_time / average_parallel_time
print(f"Speedup Factor: {speedup_factor:.2f}")
