# MPC_Trajectory_Tracking

This MPC project is inspired by Self-Driving Course, and general installation guide of the below requriements are [here](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/install_Ipopt_CppAD.md).

## Requirements
Opencv\
Eigen3\
CppAD\
IPOPT

### Installation of IPOT
- You can download  ![IPOPT](https://www.coin-or.org/download/source/Ipopt/) 3.12.7 tgz file and extract it.
- Run the below command (install_ipopt.sh is found in this repo)
 ```
~/Desktop/lpopt/Ipopt-3.12.7$ sudo ./install_ipopt.sh Ipopt-3.12.7 

 ```
- If you get an error, you need some modifications:
  - First, changed the line 31 in "get.Mumps" file in Mumps folder:
   ```
  $wgetcmd http://ftp.mcs.anl.gov/pub/petsc/externalpackages/MUMPS_${mumps_ver}.tar.gz

   ```
   - Second, changed the line 197 in "Makefile.in" file in Mumps folder:
     ```
      FFLAGS = @FFLAGS@ $(MY_FDEFS) -fallow-argument-mismatch

     ```
- Run again via install_ipopt.sh
  
### Installation of CppAD
    
```
  ~/Desktop/lpopt/Ipopt-3.12.7$ sudo apt-get install cppad

```
## Related paths and linkers

To run the code propely in Code::Blocks 20.03, you need additional linker and path:
- Search Directories:
```
../../../../../usr/include/opencv4
/home/yck/Desktop/Path Tracking Algorithms/CppRobotics/include
/usr/local/include
/usr/include
../../../../../usr/include/cppad/ipopt

```
- Linker Settings (in order):
```
../../../../../usr/lib/x86_64-linux-gnu/libopencv_calib3d.so
../../../../../usr/lib/x86_64-linux-gnu/libopencv_features2d.so
../../../../../usr/lib/x86_64-linux-gnu/libopencv_flann.so
../../../../../usr/lib/x86_64-linux-gnu/libopencv_imgproc.so
../../../../../usr/lib/x86_64-linux-gnu/libopencv_core.so
../../../../../usr/lib/x86_64-linux-gnu/libopencv_highgui.so
../../../../../usr/local/lib/libipopt.so

```




