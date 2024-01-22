import os
import tkinter as tk
from tkinter import filedialog
from tkinter import simpledialog
from tkinter import messagebox
import subprocess
import getpass


def getCurrentDirectory():
    currentFilePath = os.path.abspath(__file__)
    currentDirectory = os.path.dirname(currentFilePath)
    return currentDirectory


def getParentDirectory():
    parentDdirectory = os.path.dirname(getCurrentDirectory())
    return parentDdirectory


def webserverIsUp():
    # ip = getEnvVariable("NET_IP")
    ip = "0.0.0.0"
    port = getEnvVariable("WEB_PORT")
    try:
        result = subprocess.run(["netstat", "-tuln"], capture_output=True, text=True, check=True)
        if f"{ip}:{port}" in result.stdout:
            return True
        else:
            return False
    except:
        return False


def getEnvVariable(_key):
    try:
        with open(envFilePath, 'r') as file:
            lines = file.readlines()
        for line in lines:
            parts = line.strip().split('=')
            if len(parts) == 2 and parts[0] == _key:
                return parts[1]
        return None
    except:
        return None


def get_sudo_password():
    global sudo_password
    sudo_password = simpledialog.askstring("Sudo Password", "Enter your sudo password:", show='*')
    return sudo_password


def writeToInfobox(_text):
    output_text.config(state=tk.NORMAL)
    # output_text.delete(1.0, tk.END)
    output_text.insert(tk.END, "\n--------------\n" + _text)    
    output_text.config(state=tk.DISABLED)
    output_text.see(tk.END)  


def run_bash_script(_script_filename, _confirmation_text):
    if(webserverIsUp()):
        writeToInfobox("The platform is running. Please stop it before pefrorming any actions.")
        return
    global sudo_password

    confirmation = messagebox.askyesno("Confirm action", _confirmation_text,
                                       icon='warning', 
                                       detail="Are you sure you want to proceed?")
    if confirmation:
        # script_path = filedialog.askopenfilename(title="Select Bash Script", filetypes=[("Bash Scripts", "*.sh")])
        script_path = os.path.join(getCurrentDirectory(), _script_filename)

        if sudo_password == "":
            sudo_password = get_sudo_password()
            
        # Use echo to send the sudo password to sudo via stdin
        command = f"echo {sudo_password} | sudo -S bash {script_path} noinput"
        result = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        writeToInfobox(result.stdout.decode())
        
        if result.returncode != 0:
            writeToInfobox(result.stderr.decode())


# Function to create and pack a button with a specific parameter
def create_bash_script_button(_name, _script, _confirmation_text):
    return tk.Button(app, text=f"{_name}", command=lambda: run_bash_script(_script, _confirmation_text))


def deleteDatabase():
    if(webserverIsUp()):
        writeToInfobox("The platform is running. Please stop it before deleting the database.")
        return
        
    confirmation = messagebox.askyesno("Confirm data reset", "Are you sure you want to clear all the data?",
                                       icon='warning', 
                                       detail="All the data (including user accounts, operations, etc.) will be deleted. This action cannot be undone.")
    if confirmation:    
        try:
            dockerVolume = f"{os.path.basename(getParentDirectory())}_mysql-data"
            writeToInfobox(f"Deleting Docker volume '{dockerVolume}'")
            subprocess.run("docker rm -f db", shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            subprocess.run(f"docker volume rm {dockerVolume}", shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            envFilePath = os.path.join(getParentDirectory(), '.env')
            os.remove(envFilePath) # delete .env file
            writeToInfobox("Database was deleted successfully.")
        except Exception as e:
            writeToInfobox(f"An error occured: {e}")
    else:
        writeToInfobox(f"Aborted!")
    



# Create the main application window
app = tk.Tk()
app.title("AIDERS Tools")

envFilePath = os.path.join(getParentDirectory(), '.env')

button3 = create_bash_script_button("Patch hosts", "patch_hosts.sh", "This action will patch your hosts file to make the platform accessible via http://aiders:{port}")
button3.pack(padx=5, pady=5)

buttonResetDb = tk.Button(app, text=f"Delete Database", command=deleteDatabase)
buttonResetDb.pack(padx=5, pady=5)

button1 = create_bash_script_button("Delete media files", "delete_media.sh", "This action will delete all the media files")
button1.pack(padx=5, pady=5)

button2 = create_bash_script_button("Delete Docker data", "delete_docker_data.sh", "This action will delete all the Docker data")
button2.pack(padx=5, pady=5)

button4 = create_bash_script_button("GIT Repository Clean-up", "git_maintenance.sh", "This action will clean-up the GIT repository")
button4.pack(padx=5, pady=5)

button5 = create_bash_script_button("Nvidia Toolkit Install", "nvidia_container_install.sh", "This action will install the Nvidia container toolkit")
button5.pack(padx=5, pady=5)

button6 = create_bash_script_button("Nvidia Toolkit Enable", "nvidia_container_enable.sh", "This action will enable the Nvidia container toolkit")
button6.pack(padx=5, pady=5)

button6 = create_bash_script_button("Nvidia Toolkit Disable", "nvidia_container_disable.sh", "This action will disable the Nvidia container toolkit")
button6.pack(padx=5, pady=5)

# Create a text widget to display the output
output_text = tk.Text(app, font=("Monospace", 8), fg="orange", bg="black", height=9, width=55, state=tk.DISABLED)
output_text.pack(padx=0, pady=0)

sudo_password = ""

# Start the Tkinter event loop
app.mainloop()
