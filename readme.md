# Point to Distribution Metric
<b>Introduction</b>
<p>A novel point-to-distribution metric for PC quality assessment considering both the geometry and texture, individually and jointly. This new quality metric exploits the      scale-invariance property of the Mahalanobis distance to assess first the geometry and color point-to-distribution distortions, which are after fused to obtain a joint geometry and color quality metric.</p>
<p> Below table shows the objective-Subjective correlation performance of this metric, compared with most famous state-of-the-art metrics using the MOS scores provied in <a href="https://www.epfl.ch/labs/mmspg/downloads/quality-assessment-for-point-cloud-compression">M-PCCD</a> dataset. </p>
<table style="width:50%" align="center">
  <tr>
    <th>Type</th>
    <th>Metric</th>
    <th>SROCC</th> 
    <th>PLCC</th>
    <th>RMSE</th>
  </tr>
  <tr>
    <td>Point-to-Point</td>
    <td>D1-PSNR</td>
    <td>79.1</td>
    <td>77.7</td>
    <td>0.857</td>
  </tr>
  <tr>
    <td>Point-to-Point</td>
    <td>GH 98% PSNR</td>
    <td>86.9</td>
    <td>84.6</td>
    <td>0.726</td>
  </tr>
  <tr>
    <td>Point-to-Point</td>
    <td>RA-PSNR (APD<sub>10</sub>)</td>
    <td>90.2</td>
    <td>88.8</td>
    <td>0.626</td>
  </tr>
  <tr>
    <td>Point-to-Point</td>
    <td>Y-PSNR</td>
    <td>66.2</td>
    <td>67.1</td>
    <td>1.009</td>
  </tr>
  <tr>
    <td>Point-to-Plane</td>
    <td>D2-PSNR</td>
    <td>83.8</td>
    <td>80.5</td>
    <td>0.808</td>
  </tr>
  <tr>
    <td>Point-to-Plane</td>
    <td>GH 98% PSNR</td>
    <td>87.9</td>
    <td>84.3</td>
    <td>0.731</td>
  </tr>
  <tr>
    <td>Point-to-Plane</td>
    <td>RA-PSNR (APD<sub>10</sub>)</td>
    <td>89.9</td>
    <td>88.9</td>
    <td>0.622</td>
  </tr>
  <tr>
    <td>Feature-Based</td>
    <td>PointSSIM</td>
    <td>91.8</td>
    <td>92.6</td>
    <td>0.514</td>
  </tr>
  <tr>
    <td>Feature-Based</td>
    <td>d<sub>gc</sub></td>
    <td>92.0</td>
    <td>90.4</td>
    <td>0.585</td>
  </tr>
  <tr>
    <td>Feature-Based</td>
    <td>H<sup>Y</sup><sub>L2</sub></td>
    <td>88.4</td>
    <td>85.3</td>
    <td>0.710</td>
  </tr>
  <tr>
    <td>Feature-Based</td>
    <td>PCM<sub>RR</sub>(MCCV)</td>
    <td>D90.7oe</td>
    <td>90.2</td>
    <td>0.573</td>
  </tr>
  <tr>
    <td>Point-to_Distribution</td>
    <td>P2D-G</td>
    <td>89.0</td>
    <td>87.3</td>
    <td>0.663</td>
  </tr>
  <tr>
    <td>Point-to_Distribution</td>
    <td>LogP2D-G</td>
    <td>89.0</td>
    <td>87.3</td>
    <td>0.664</td>
  </tr>
  <tr>
    <td>Point-to_Distribution</td>
    <td>P2D-Y</td>
    <td>89.3</td>
    <td>90.5</td>
    <td>0.578</td>
  </tr>
  <tr>
    <td>Point-to_Distribution</td>
    <td>LogP2D-Y</td>
    <td>89.3</td>
    <td>90.7</td>
    <td>0.574</td>
  </tr>
  <tr>
    <td>Point-to_Distribution</td>
    <td>P2D-JGY</td>
    <td><b>93.8</b></td>
    <td><b>92.9</b></td>
    <td>0.503</td>
  </tr>
  <tr>
    <td>Point-to_Distribution</td>
    <td>LogP2D-JGY</td>
    <td><b>93.8</b></td>
    <td><b>92.9</b></td>
    <td><b>0.502</b></td>
  </tr>
</table>
   
   
<b>Compiling instructions</b>

   Download <a href="http://eigen.tuxfamily.org/index.php?title=Main_Page">Eigen</a> C++ template library. 
   Rest of the dependencies are available in dependencies folder.
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
   
   * Note that distSize has a default value of 31. (Values more than 100 are not allowed.)
   
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

   If you are using this metric, please cite the following <a href="https://ieeexplore.ieee.org/document/9143408?title=Main_Page">publication</a>:
   
A. Javaheri, C. Brites, F. Pereira and J. Ascenso, "Mahalanobis Based Point to Distribution Metric for Point Cloud Geometry Quality Evaluation," in IEEE Signal Processing Letters, vol. 27, pp. 1350-1354, 2020, doi: 10.1109/LSP.2020.3010128.
