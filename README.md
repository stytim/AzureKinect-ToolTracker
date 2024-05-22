# IR Retro-reflective Marker Tracking with Azure Kinect

[![GitHub release](https://img.shields.io/github/release/stytim/AzureKinect-ToolTracker.svg?style=flat-square)](https://github.com/stytim/AzureKinect-ToolTracker/releases)

This project leverages the versatility of Microsoft Azure Kinect cameras, enabling high-quality tracking of passive IR sphere markers without the need for large and expensive equipment like NDI trackers.

<p align="center">
	<img width="80%" src="image/overview.gif">
</p>


## Project Highlights
* **Compact and Cost-Effective**: Break free from the constraints of traditional tracking systems. Our project utilizes the small, affordable Azure Kinect camera for efficient tracking.
* **Versatile and User-Friendly**: Efficiently handle simultaneous tracking of various markers. Enjoy robust features such as multiple marker tracking, occlusion resistance, and support for various marker types and sizes.
* **Smooth and Stable Tracking**: Integrate Kalman and low-pass filters for stable tracking.
* **Enhanced Communication and Compatibility**: Utilize UDP messaging for transmitting tracking results and support for NDI .rom files
* **Simple Calibration**: A simple marker array calibration is provided if the marker configuration is not known beforehand
* **Cross-Platform Availability**: Works across Linux and Windows environments.

## Installation
**Quick Start with Precompiled Binary (Windows):** Download the latest precompiled binary from our releases section for immediate use.

**Building from Source:** For those interested in customizing or contributing to the project, please see the build instructions below.

## Prerequisites

* Azure Kinect SDK
* OpenCV 4
* CMake 

## Building
Ensure all the prerequisites are installed before proceeding with the build process.

### For Linux:

```bash
mkdir build && cd build
cmake ..
make
```
### For Windows
On Windows, you'll need to use CMake to generate build files specific to your platform (e.g., Visual Studio). After generating these files, you can build the project using your chosen IDE or build system.
Note: I have only tested this build process with Visual Studio 2022.


## Running

### For Linux and Windows:
```bash
./ir-tracking-app
```
### Offline Processing from Recording
```bash
./ir-tracking-app -i path_to_recording.mkv
```

## Usage Guide

Upon launching the K4A Tool Tracker, the application will attempt to load previously defined tool configurations from the 'Tools' directory. Follow the steps below to set up your environment and begin tracking:

### 1. Tool Configuration:

* Number of Tools: Specify how many tools you wish to track.
* Tool Details: For each tool, define the following:
  - Number of Spheres: Indicate how many spheres are attached to each tool.
  - Sphere Radius: Enter the radius for each sphere to ensure precise tracking.
  - Tool Name: Assign a unique name for easy identification.
  - Sphere Positions: Input the positions of each sphere relative to the tool's coordinate system.

* Tool Configuration:
  - Load ROM (Optional): To use pre-defined configurations, click "Load ROM" and choose the relevant NDI ROM file
  - Calibrate Tool (Optional): If the configuration is unknown, ensure no reflective objects are in the camera's view, then calibrate by clicking "Calibrate Tool."
  - Add Tool Definition: Finalize the setup and enable tracking by clicking "Add Tool Definition."
  - Repeat this process for multiple tools as needed.

### 2. Network Configuration

* Input the "IP Address" and "Port" for networked UDP messaging.
* Transmission Rate: Confirm the "Frequency" of tracking updates to suit your needs.
* UDP Enable: Ensure the "UDP" checkbox is selected to activate network transmission.

### 3. Save Tracking Results Locally to CSV

* Rate: Input the "Frequency" of writing the CSV file.
* Duration: Input how long in seconds to save the file
* Save Location: Click "Save To" to select the name and location for the CSV file.
* Record Enable: Ensure the "Record" checkbox is selected to activate recording to file.

### 4. Start Tracking
* Initiate Tracking: With all parameters set, initiate tracking by clicking "Start Tracking."
* Real-Time Adjustments: You can tweak the Tracking Parameters in real-time to adapt to varying conditions.
  - Infrared Sensitivity: Adjust the "IR Threshold" to optimize infrared tracking sensitivity.
  - Pixel Range: Set the "Min Px" and "Max Px" to define the acceptable pixel range for detection.
* Results Monitor: The application will display the position and orientation data for each configured tool.

### Additional Resources
A sample Python and a C# Unity scripts is provided for receiving and processing the tracking data via UDP.

## Contributing
Contributions are welcome! When submitting a pull request, please include a description of your improvements and reference any relevant issue numbers.

## Frequently Asked Questions (FAQs)
### Can I use Intel RealSense Cameras for this project?

Yes, please refer to the [RealSense Tool Tracker](https://github.com/stytim/RealSense-ToolTracker)


## License and Citation

This project is distributed under the MIT License.

The core idea of the tracking algorithm is based on the following publication:

```bibtex
@ARTICLE{10021890,
  author={Martin-Gomez, Alejandro and Li, Haowei and Song, Tianyu and Yang, Sheng and Wang, Guangzhi and Ding, Hui and Navab, Nassir and Zhao, Zhe and Armand, Mehran},
  journal={IEEE Transactions on Visualization and Computer Graphics}, 
  title={STTAR: Surgical Tool Tracking using Off-the-Shelf Augmented Reality Head-Mounted Displays}, 
  year={2023},
  volume={},
  number={},
  pages={1-16},
  doi={10.1109/TVCG.2023.3238309}}

```
A significant portion of the tracking code is adapted from Andreas Keller's work:
```BibTeX
@misc{keller2023hl2irtracking,
  author =       {Andreas Keller},
  title =        {HoloLens 2 Infrared Retro-Reflector Tracking},
  howpublished = {\url{https://github.com/andreaskeller96/HoloLens2-IRTracking}},
  year =         {2023}
}
```


<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=stytim/Azure-ToolTracker&type=Date&theme=dark" />
  <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=stytim/AzureKinect-ToolTracker&type=Date" />
  <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=stytim/AzureKinect-ToolTracker&type=Date" />
</picture>
