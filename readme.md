# Point to Distribution Metric

<b>Compiling instructions</b>

   Download <a href="http://eigen.tuxfamily.org/index.php?title=Main_Page">Eigen</a> C++ template library. 
   rest of the dependencies are available in dependencies folder.
   CMakeLists.txt is provided for cmake to generate makefiles. General
   practice using cmake should be followed in order to compile the
   program. Suggested steps to compile under Debug mode are shown below.
   ```console
   :$ cd /path/to/root/folder/of/the/source/code/
   ```
   ```console
   :$ ls changes.txt  readme.txt  source  test
   ```
   
   ```console
   :$ mkdir debug
   ```
   
   ```console
   :$ cd debug
   ```
   ```console
   :$ cmake -DCMAKE_BUILD_TYPE=Debug ../source
   ```
   
   You may want to specify the location of a pariticular Boost version,
   e.g., via -DBOOST_ROOT=/path/to/boost_1_63_0

   ```console
   :$ make
   "pc_error_d" to be generated under ./test folder
   ```
   
<b> usage </b>

   ```console
   ./test/pc_error  [--help] [-c config.cfg] [--parameter=value]
   ```

   The metrics takes as input a PLY files: source(A), test(B) and size of the distribution in target PC 
   and compute the point-to-distribution distance between A and B.

   The outputs are writing in the terminal as trace and can be catch in log files. 

   Example:
   
   ```console
   ./test/pc_error 
   --fileA=./queen/frame_0010.ply 
   --fileB=./S22C2AIR01_queen_dec_0010.ply 
   --distSize=31
   --resolution=1023
   ```
   
   * Note that distSize has a default value of 31
   
   if you are also interested in MPEG metric D1 and D2 use the following example:
   ```console
   ./test/pc_error 
   --fileA=./queen/frame_0010.ply 
   --fileB=./S22C2AIR01_queen_dec_0010.ply 
   --inputNorm=./queen_n/frame_0010_n.ply
   --distSize=31 
   --color=1 
   --resolution=1023
   ```

<b> Reference </b>

   If you are using this metric, please cite the following publication:
   
   A. Javaheri, C. Brites, F. Pereira, J. Ascenso, “Mahalanobis based point to distribution metric for point cloud geometry quality evaluation,” IEEE Signal Processing Letters,    submitted on Apr. 2020.
