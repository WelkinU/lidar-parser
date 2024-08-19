# lidar-parser

This repository is a Python parser for lidar data. Intent is to be an efficient pure Python implementation that covers a wide variety of rotating 3D lidars, with minimal dependency on other libraries.

Functionality:
- Parse lidar packets from the input data
- Autodetect lidar type, auto-load lidar calibration
- Decode the compressed lidar data into cartesian coordinates (Velodyne /Hesai: XYZI, Ouster: XYZIA)
- Assemble lidar data into full point clouds

**Currently supported lidars:**

Velodyne: `VLS-128`, `HDL-64`, `VLP-32c`, `Velodyne-32e`, `VLP-16`

**Currently supported input formats:**

Currently, PCAP is the only supported input format. If you want to use a different format, make a custom reader class and specify it via the `reader_class` keyword arg - see the associated section below.

## Example Usage

``` python
from readers import PCAPReader
from velodyne import VelodyneReader
from visualize import plot_cloud

pcap_reader = PCAPReader("path/to/your/file.pcap")
reader = VelodyneReader(reader = pcap_reader)

for lidar in reader:      #lidar is Nx4 numpy array in XYZI format (Ouster is Nx5)
    plot_cloud(lidar)     #3d scatter plot the point cloud
    break                 #plot only the first full point cloud
```

## Installation

- Python >= 3.6
- Libraries: numpy, matplotlib

## Create custom packet reader

If you want to make a custom file reader, make a class that meets the following requirements:
- Must have a `self.endian` variable defined as either "little" or "big"
- Must implement a `__iter__()` method defined that yields raw lidar data with the packet header stripped

**Example skeleton class**
``` python
class YourReader:

    def __init__(self, ):

        # make sure to have an endian value set, either "little" or "big"
        self.endian = "little"
    
    def __iter__(self):

        # this function should implement a generator that for each data packet, strip the header and yield only the lidar data
```

**Example usage of custom reader**

``` python
from velodyne import VelodyneReader

reader = VelodyneReader(reader = YourReader())

for lidar in reader:
    #do something with your lidar data
```

## Notes on Ouster Lidar Parsing

- Current support only for data packet format Single Return Profile ie. `RNG19_RFL8_SIG16_NIR16` [See docs here](https://static.ouster.dev/sensor-docs/image_route1/image_route3/sensor_data/sensor-data.html#rng19-rfl8-sig16-nir16-return-profile) for more info
