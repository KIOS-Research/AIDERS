import os
import time
import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import subprocess
import select

container_names = ["db", "web", "ws", "ros", "mav", "lsc", "cv", "alg", "ccd", "rtmp", "nginx", "geo", "odm"]
line_colors = ["#0b6ee2", "#67bd4a", "#6d3537", "orange", "#ea7039", "#ef418c", "purple", "#00acac", "grey", "red", "#fd3da6", "#413aa1", "grey", "grey"]

container_cpu_values = {}
for name in container_names:
    container_cpu_values[name] = [0 for _ in range(60)]

total_cpu_values = [0 for _ in range(60)]

last_draw_timestamp = time.time()

def container_is_running(container_name):
    command = f"docker ps --format '{{{{.Names}}}}' | grep {container_name}"
    result = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return result.returncode == 0 and container_name in result.stdout

def update_output(output_text):
    global container_cpu_values
    try:
        # Run the docker stats command in a subprocess and capture the output
        command = "docker stats --no-stream"  # "--no-stream" flag prevents the command from running in interactive mode
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, text=True, bufsize=1)

        # Clear previous output
        output_text.delete("1.0", tk.END)
        output_text.insert(tk.END, f"SERVICE     CPU         RAM            BLOCK I/O\n")
        output_text.insert(tk.END, f"--------------------------------------------------------\n")
        column_chars = 12

        while True:
            # Check if there's any new output to read from the subprocess
            readable, _, _ = select.select([process.stdout], [], [], 0.1)

            if process.stdout in readable:
                output_line = process.stdout.readline()
                if not output_line:
                    break

                columns = output_line.split()
                container_name = columns[1]

                if container_name in container_names:
                    cpu = columns[2]
                    cpu_value = float(cpu[:-1])

                    container_cpu_values[container_name].append(cpu_value)
                    container_cpu_values[container_name] = container_cpu_values[container_name][-60:]  # Keep the last 60 values                     

                    ram = columns[3]
                    try:
                        ram_value = float(ram[:-3])
                        ram_units = ram[-3:]
                        if ram_units == "KiB":
                            ram_value = ram_value / 1000000
                        if ram_units == "MiB":
                            ram_value = ram_value / 1000                            
                        ram_value = float(round(ram_value, 3))
                        ram_sting = f"{ram_value} GB"
                    except:
                        ram_sting = ""

                    block_i = columns[10]
                    block_o = columns[12]

                    spaces_string = " " * (column_chars - len(container_name))
                    spaces_string2 = " " * (column_chars - len(cpu))
                    spaces_string3 = " " * ((column_chars - len(ram_sting)) + 3)
                    output_text.insert(tk.END, f"{container_name}{spaces_string}{cpu}{spaces_string2}{ram_sting}{spaces_string3}{block_i}/{block_o} \n")

    except subprocess.CalledProcessError as e:
        output_text.delete("1.0", tk.END)  # Clear any previous output
        output_text.insert(tk.END, f"Error: {e}")

    # Schedule the function to run again after a delay (in milliseconds)
    output_text.after(500, update_output, output_text)  # Run again after 500 milliseconds

def show_time_series_chart():
    global container_cpu_values
    global total_cpu_values
    global output_text

    lines = []

    def update_graph():
        global total_cpu_values
        global last_draw_timestamp

        total_cpu = 0

        for container_name in container_names:
            total_cpu = total_cpu + container_cpu_values[container_name][59]

        total_cpu_values.append(round(total_cpu, 2))
        total_cpu_values = total_cpu_values[-60:]


        for i, line in enumerate(lines):
            if (i == len(container_names)):
                # TOTALS line
                line.set_data(range(len(total_cpu_values)), total_cpu_values)
                line.set_visible(True)
            else:
                if container_is_running(container_names[i]):
                    line.set_data(range(len(container_cpu_values[container_names[i]])), container_cpu_values[container_names[i]])
                    line.set_visible(True)
                    # total_cpu = total_cpu + container_cpu_values[container_names[i]][59]
                else:
                    line.set_data(range(len(container_cpu_values[container_names[i]])), 0)
                    line.set_visible(False)        

        # print(total_cpu)
        # Calculate the average
        non_zero_cpu_values = [value for value in total_cpu_values if value != 0]
        if non_zero_cpu_values:
            avg_cpu = sum(non_zero_cpu_values) / len(non_zero_cpu_values)
        else:
            avg_cpu = 0

        draw_timestamp = time.time()
        loop_seconds = draw_timestamp - last_draw_timestamp
        last_draw_timestamp = draw_timestamp

        # avg_cpu = sum(total_cpu_values) / len(total_cpu_values)
        output_text.insert(tk.END, f"--------------------------------------------------------\n")
        output_text.insert(tk.END, f" TOTAL: {round(total_cpu, 1)}%           AVG: {round(avg_cpu, 1)}%                 {round(loop_seconds, 1)}s")

        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()

        # Schedule the function to update the graph after a delay (in milliseconds)
        fig.canvas.get_tk_widget().after(500, update_graph)

    # Create the main application window
    window = tk.Tk(className="AIDERS Monitoring")
    window.title("AIDERS Monitoring")
    icon = tk.PhotoImage(file=f"{getCurrentDirectory()}/icon.png")
    window.iconphoto(False, icon)

    # Create a Text widget to display the output
    output_text = tk.Text(window, wrap=tk.WORD, width=56, height=len(container_names)+4)
    output_text.configure(fg="light grey", bg="black")
    output_text.pack(padx=0, pady=0)

    # Start updating the output in real-time
    update_output(output_text)


    # plt.style.use('dark_background')

    # Create the line graph
    fig, ax = plt.subplots(figsize=(4.72, 3))
    ax = plt.gca()
    fig.set_facecolor('lightgrey')
    
      # ax.set_xlabel('Time')
    ax.set_xticklabels([])
    # ax.set_ylabel('CPU Usage (%)')
    ax.set_title('CPU Usage (%)')

    # create the line objects
    total_containers = len(container_names)
    for i, name in enumerate(container_names):
        line_style = "-"
        # if i == total_containers - 1:
        if i > 9:
            line_style = ":"
        line, = ax.plot([], [], marker='o', linestyle=line_style, label=name, linewidth=2, color=line_colors[i])
        line.set_markersize(0)
        lines.append(line)

    # TOTAL line
    line, = ax.plot([], [], marker='o', linestyle="--", label="TOTAL", linewidth=1, color="grey")
    line.set_markersize(0)
    lines.append(line)

    # legend
    ax.legend(fontsize=8, loc='upper left', bbox_to_anchor=(1.02, 1), borderaxespad=0.)
    
    ax.grid(axis='y', linestyle='dotted')

    plt.tight_layout(rect=(0, 0, 1, 1))

    # Convert the Matplotlib figure to a Tkinter canvas
    canvas = FigureCanvasTkAgg(fig, master=window)
    canvas.get_tk_widget().pack(padx=0, pady=0)  # <-- Position the canvas in the main window
    
    update_graph()  # Start updating the graph

    # Start the Tkinter main loop
    window.mainloop()


def getCurrentDirectory():
    currentFilePath = os.path.abspath(__file__)
    currentDirectory = os.path.dirname(currentFilePath)
    return currentDirectory



if __name__ == "__main__":
    show_time_series_chart()
