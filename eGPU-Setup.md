## eGPU Setup
Full eGPU Setup Guide — RTX 5070 Ti on Ubuntu 24.04 (Framework Laptop)

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

### 4.Prepare for Installation
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

