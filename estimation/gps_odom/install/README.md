### Installing GeographicLib

1. Download version 1.35 of GeographicLib from [SourceForge](http://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.36.tar.gz/download).
2. `cd` into the directory you downloaded, and apply `fixgeo.patch` using the following command:
  ```bash
  patch -p1 < fixgeo.patch
  ```
3. You should see output indicating that a documentation file has been patched.
4. Build and install GeographicLib using the following commands (assuming you are in the `GeographicLib-1.35` directory):
  ```bash
  mkdir build
  cd build
  cmake ..
  make -j4
  sudo make install
  ```
5. Done. **gps_odom** should now build without issue.
