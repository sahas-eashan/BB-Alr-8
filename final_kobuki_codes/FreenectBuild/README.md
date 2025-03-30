# Installing Freenect

## 1. Update the System

```sh
sudo apt-get update
sudo apt-get upgrade
```

## 2. Install Dependencies

```sh
sudo apt-get install git-core cmake freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev
```

## 3. Clone the libfreenect Repository

```sh
git clone https://github.com/OpenKinect/libfreenect
```

> **Note:** If you are using the Python wrapper, you need to update `CMakeLists.txt` in `wrappers/python`. Change references to `cython` to `cython3`. (The worked cmakelist is in this folder)

## 4. Install libfreenect

```sh
cd libfreenect
mkdir build
cd build
cmake -L .. 
# Use the following command if using Python wrapper
# cmake .. -DBUILD_PYTHON3=ON
make
sudo ldconfig /usr/local/lib64/
```

## 5. Enable Kinect Access for Non-Root Users

```sh
sudo adduser $USER video
sudo adduser $USER plugdev
```

## 6. Create Udev Rules

```sh
sudo nano /etc/udev/rules.d/51-kinect.rules
```

Add the following rules:

```sh
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
# ATTR{product}=="Xbox NUI Audio"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
# ATTR{product}=="Xbox NUI Camera"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
```

## 7. Fixing "LIBUSB\_ERROR\_IO" Issue

```sh
cd libfreenect
python3 ./src/fwfetcher.py
```

Move the `audio.bin` file to the installed location:

```sh
sudo mv ./audios.bin /usr/local/share/libfreenect/
```

## 8. Test the Installation

```sh
freenect-micview
freenect-camtest 
freenect-glview
```

---

# Python Wrapper - Using Freenect with Python

## 1. Install Required Libraries

```sh
sudo apt-get install cython3
sudo apt-get install python-dev-is-python3
sudo apt-get install python3-numpy
```

## 2. Modify `setup.py`

Modify the file `libfreenect/wrappers/python/setup.py` with the following content:

```python
#!/usr/bin/env python
from setuptools import setup, Extension
import re
import numpy as np

def get_cython_version():
    """
    Returns:
        Version as a pair of ints (major, minor)
    Raises:
        ImportError: Can't load cython or find version
    """
    import Cython
    try:
        version = Cython.__version__
    except AttributeError:
        version = Cython.Compiler.Main.version
    match = re.search('^([0-9]+)\.([0-9]+)', version)
    try:
        return [int(g) for g in match.groups()]
    except AttributeError:
        raise ImportError

try:
    cython_version = get_cython_version()
    if cython_version[0] == 0 and cython_version[1] < 13:
        raise ImportError
    from Cython.Distutils import build_ext
    source_ext = '.pyx'
    cmdclass = {'build_ext': build_ext}
except ImportError:
    source_ext = '.c'
    cmdclass = {}

ext_modules = [
    Extension("freenect", ["freenect" + source_ext],
              libraries=['usb-1.0', 'freenect', 'freenect_sync'],
              runtime_library_dirs=['/usr/local/lib', '/usr/local/lib64', '/usr/lib/'],
              extra_compile_args=['-fPIC', '-I', '../../include/',
                                  '-I', '/usr/include/libusb-1.0/',
                                  '-I', '/usr/local/include/libusb-1.0',
                                  '-I', '/usr/local/include',
                                  '-I', '../c_sync/',
                                  '-I', np.get_include()])
]

setup(name='freenect',
      cmdclass=cmdclass,
      ext_modules=ext_modules)
```

## 3. Install Python Wrapper

```sh
sudo python setup.py install
```

**Happy Coding!**

