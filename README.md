# MPC_Trajectory_Tracking

This MPC project is inspired by UDACITY Self-Driving Course, and general installation guide of the below requriements are [here](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/install_Ipopt_CppAD.md).

## Requirements
Opencv\
Eigen3\
CppAD\
IPOPT

### Installation of IPOT
- You can download  ![IPOPT](https://www.coin-or.org/download/source/Ipopt/) 3.17.12 tgz file and extract it.
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




