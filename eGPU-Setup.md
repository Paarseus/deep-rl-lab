## eGPU Setup
Full eGPU Setup Guide — RTX 5070 Ti on Ubuntu 24.04 (Framework Laptop)

https://developer.nvidia.com/blog/accelerating-machine-learning-on-a-linux-laptop-with-an-external-gpu/

>Ubuntu 24.04 installed <br>
>Secure Boot disabled in BIOS <br>
>eGPU plugged in via Thunderbolt 4/USB4

### 1.Verify eGPU Connection
```Shell
$ lspci | grep -i nvidia
```
Result:
```Shell
57:00.0 VGA compatible controller: NVIDIA Corporation Device 2c05 (rev a1)
```
If not detected:
    Reboot with eGPU plugged in
    Check boltctl for Thunderbolt status
    Ensure GPU has power

### 2.Add Graphics Drivers PPA
```Shell
$ sudo add-apt-repository ppa:graphics-drivers/ppa
$ sudo apt update
```

### 3.Download Official NVIDIA Driver

From:
https://www.nvidia.com/Download/index.aspx
Choose:
    GPU: RTX 5070 Ti
    OS: Linux 64-bit
    Download .run file (e.g. NVIDIA-Linux-x86_64-570.144.run)
### 3.1 NVIDIA Driver Package Installation (Optional)
```shell
$ sudo apt install nvidia-driver-570-open -y
$ sudo apt update
```

### 3.2 Install CUDA 
Install CUDA 12.8 for proper GPU support:
```shell
# Download and install the CUDA keyring
$ wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
$ sudo dpkg -i cuda-keyring_1.1-1_all.deb

# Update and install CUDA toolkit
$ sudo apt update
$ sudo apt install cuda-toolkit-12-8 -y
```

### 3.3 Install Thunderbolt Utilities
Install tools to manage Thunderbolt connections:
```shell
$ sudo apt install bolt thunderbolt-tools
```

### 3.4 Configure Thunderbolt Connection
```shell
# Check Thunderbolt controller status
$ sudo boltctl

# Connect your eGPU to the Thunderbolt port on your laptop
# Then authorize the device
$ sudo boltctl enroll [UUID or device name]

# Verify the connection
$ lspci | grep -i nvidia
```
### 3.5 Configure NVIDIA Settings for External GPU
```shell
# Configure X server to allow external GPUs
$ sudo nvidia-xconfig --allow-external-gpus

# Reboot your system
$ sudo reboot
```
### 3.6 Verify NVIDIA Installation
After rebooting, check if your GPU is properly recognized:
```shell
# Check if NVIDIA driver is loaded
$ nvidia-smi

# Check NVIDIA settings
$ nvidia-settings
```

### 4.Prepare for Installation (also used for uninstalling)
Clean up any previous drivers:
```Shell
$ sudo apt purge '^nvidia-.*' -y
$ sudo apt autoremove -y
```

Then reboot:
```Shell
$ sudo reboot
```

### 5.Run the .run Installer
```Shell
$ cd ~/Downloads
$ chmod +x nvidia.run
$ sudo ./nvidia.run
```
### 6.Reboot and Confirm Installation
```Shell
$ sudo reboot
```
Then run:
```Shell
$ nvidia-smi
```
You should see your RTX 5070 Ti listed.

### 7.Offload Apps to eGPU (No Reboot Needed)
```Shell
$ __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia <your-app>
```
Examples:
```Shell
$ __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia blender
$ __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia /path/to/UnrealEditor
```
Test with:
```Shell
$ glxinfo | grep "OpenGL renderer"
```

### Live GPU Monitoring with watch

Run this:
```Shell
$ watch -n 1 nvidia-smi
```


### UPDATE
#### 1. Official NVIDIA eGPU Documentation <br>
https://developer.nvidia.com/blog/accelerating-machine-learning-on-a-linux-laptop-with-an-external-gpu/ <br>

#### 2. Disable Nouveau Kernel Driver <br>
https://askubuntu.com/questions/841876/how-to-disable-nouveau-kernel-driver <br>

#### 3. NVIDIA Prime Package
```shell
$ sudo apt install nvidia-prime
$ sudo prime-select query
$ sudo prime-select nvidia/on-demand/intel
$ sudo reboot
```
#### 4. "xrandr" for Monitor Configuration
```shell
$ xrandr --listproviders
```

#### 5. Errors
```shell
docker: Error response from daemon: unknown or invalid runtime name: nvidia
```
Havent found a solution yet! From reading NVIDIA container toolkit github, found out the container already has the CUDA Drivers inside so im guessing installing the both caused the problem.

