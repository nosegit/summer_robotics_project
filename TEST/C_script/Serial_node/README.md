
# Arduino Serial Node Project

This is a modern C++ project utilizing CMake and `vcpkg` in manifest mode to manage dependencies cleanly and reproducibly. It relies on `libserial` to communicate with an Arduino device over serial.

## Prerequisites

Before building, ensure your environment has the following installed:
*   **C++ Compiler**: Supporting C++20 (GCC or Clang).
*   **CMake**: Version 3.20 or higher.
*   **Ninja**: The build system specified in the preset.
*   **vcpkg**: Microsoft's C++ package manager. Ensure your `CMakePresets.json` correctly points to your `vcpkg.cmake` toolchain file (e.g., `~/vcpkg/scripts/buildsystems/vcpkg.cmake`).

## Build Instructions

This project uses CMake Presets to simplify the configuration and build process. You do not need to manually install dependencies; `vcpkg` handles this automatically during configuration.

### 1. Configure the Project
This step initializes the build environment and automatically triggers `vcpkg` to download, build, and isolate `libserial` locally.
```bash
cmake --preset default

```

### 2. Build the Project

Compile the source code into a runnable executable using Ninja.

```bash
cmake --build build

```

## Running the Application

Once the build successfully completes, the executable is placed in the root of the `build` directory. Execute it using:

```bash
./build/TestArduino

```

---

## WSL Users: Serial Hardware Setup

If you are developing inside Windows Subsystem for Linux (WSL), the virtual machine cannot natively see physical USB devices. You must use `usbipd-win` to bridge the connection.

### 1. Pass the USB Device to WSL

From a standard Windows PowerShell prompt (use an Administrator prompt if binding for the very first time):

```powershell
# Find your device's BUSID (e.g., 1-3)
usbipd list

# Bind the device (Admin required only once per port/device)
usbipd bind --busid <BUSID>

# Attach the device to WSL
usbipd attach --wsl --busid <BUSID>

```

### 2. Grant Serial Permissions

Inside your WSL terminal, ensure your user has permission to read and write to serial ports (usually `/dev/ttyACM0` or `/dev/ttyUSB0`).

```bash
sudo usermod -a -G dialout $USER

```

*Note: You must completely close and reopen your WSL terminal for the group change to take effect.*

### 3. Detaching the Device

When you are done testing and want Windows to have access to the Arduino again, run this in PowerShell:

```powershell
usbipd detach --busid <BUSID>

```