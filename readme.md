# Point-to-Distribution Point Cloud Quality Metric
<b>Introduction</b>
<p>A novel point-to-distribution metric for PC quality assessment considering both the geometry and texture jointly. This new quality metric exploits the scale-invariance property of the Mahalanobis distance to assess first the geometry and color point-to-distribution distortions, which are after fused to obtain a joint geometry and color quality metric.</p>
<p> The software reports a joint geometry and luminance mean mahalanobis distance metric (<b>P2D-JGY</b>) and a joint geometry and luminance mean mahalanobis logarithmic distance metric (<b>LogP2D-JGY</b>). The software also reports these two distances for geoemtry and color (any color component) individually. However, the best performance acquired measuring color and geometry distortions jointly. All these distances are reported in the table below. The best that should be used for any further evaluation is shown in bold. </p>

<table style="width:50%" align="center">
  <tr>
    <th>Metric Name</th>
    <th>Software Output Name</th>
  </tr>
  <tr>
    <td><b>P2D-JGY</b></td>
    <td>mmdJoint</td>
  </tr>
  <tr>
    <td><b>LogP2D-JGY</b></td>
    <td>mmdJoint, LOG</td>
  </tr>
  <tr>
    <td>P2D-G</td>
    <td>mmd</td>
  </tr>
  <tr>
    <td>LogP2D-G</td>
    <td>mmd, LOG</td>
  </tr>
  <tr>
    <td>P2D-Y</td>
    <td>mmdColor[0]</td>
  </tr>
  <tr>
    <td>LogP2D-Y</td>
    <td>mmdColor[0], LOG</td>
  </tr>
  <tr>
    <td>P2D-Cb</td>
    <td>mmdColor[1]</td>
  </tr>
  <tr>
    <td>LogP2D-Cb</td>
    <td>mmdColor[1], LOG</td>
  </tr>
  <tr>
    <td>P2D-Cr</td>
    <td>mmdColor[2]</td>
  </tr>
  <tr>
    <td>LogP2D-Cr</td>
    <td>mmdColor[2], LOG</td>
  </tr>
</table>
</p>  
 
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

   If PCs don't have color or you are not interested in color (and joint) metric(s), set --color=0 (default). 

   Example:
   
   ```console
   ./test/pc_error 
   --fileA=./queen/frame_0010.ply 
   --fileB=./S22C2AIR01_queen_dec_0010.ply 
   --distSize=31
   --color=1
   --resolution=1023
   ```
   
   * Note that distSize has a default value of 31. (Values more than 100 are not allowed.)
   
   if you are also interested to get point-to-plane metric results (MPEG D2), you have to include also PC normals as in the following example:
   ```console
   ./test/pc_error 
   --fileA=./queen/frame_0010.ply 
   --fileB=./S22C2AIR01_queen_dec_0010.ply 
   --inputNorm=./queen_n/frame_0010_n.ply
   --distSize=31 
   --color=1 
   --resolution=1023
   ```
<b> Performance </b>
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
    <td><a href="https://ieeexplore.ieee.org/abstract/document/9123087/">GH 98% PSNR</a></td>
    <td>86.9</td>
    <td>84.6</td>
    <td>0.726</td>
  </tr>
  <tr>
    <td>Point-to-Point</td>
    <td><a href="https://ieeexplore.ieee.org/abstract/document/9191233">RA-PSNR (APD<sub>10</sub>)</a></td>
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
    <td><a href="https://ieeexplore.ieee.org/abstract/document/9123087/">GH 98% PSNR</a></td>
    <td>87.9</td>
    <td>84.3</td>
    <td>0.731</td>
  </tr>
  <tr>
    <td>Point-to-Plane</td>
    <td><a href="https://ieeexplore.ieee.org/abstract/document/9191233">RA-PSNR (APD<sub>10</sub>)</a></td>
    <td>89.9</td>
    <td>88.9</td>
    <td>0.622</td>
  </tr>
  <tr>
    <td>Feature-Based</td>
    <td><a href="https://ieeexplore.ieee.org/abstract/document/9106005">PointSSIM</a></td>
    <td>91.8</td>
    <td>92.6</td>
    <td>0.514</td>
  </tr>
  <tr>
    <td>Feature-Based</td>
    <td><a href="https://ieeexplore.ieee.org/abstract/document/9123089">d<sub>gc</sub></a></td>
    <td>92.0</td>
    <td>90.4</td>
    <td>0.585</td>
  </tr>
  <tr>
    <td>Feature-Based</td>
    <td><a href="https://ieeexplore.ieee.org/abstract/document/9123089">H<sup>Y</sup><sub>L2</sub></a></td>
    <td>88.4</td>
    <td>85.3</td>
    <td>0.710</td>
  </tr>
  <tr>
    <td>Feature-Based</td>
    <td><a href="https://ieeexplore.ieee.org/abstract/document/9198142">PCM<sub>RR</sub>(MCCV)</a></td>
    <td>90.7</td>
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

<b> Reference </b>

<p>If you are using this metric, please cite the following publications:</p>

A. Javaheri, C. Brites, F. Pereira and J. Ascenso, <a href="https://arxiv.org/abs/2108.00054">"A Point-to-Distribution Joint Geometry and Color Metric for Point Cloud Quality Assessment,"</a> <i>arXiv preprint arXiv:2108.00054</i>, August 2021.

A. Javaheri, C. Brites, F. Pereira and J. Ascenso, <a href="https://ieeexplore.ieee.org/document/9143408?title=Main_Page">"Mahalanobis Based Point to Distribution Metric for Point Cloud Geometry Quality Evaluation,"</a> <i>IEEE Signal Processing Letters</i>, vol. 27, pp. 1350-1354, 2020, doi: 10.1109/LSP.2020.3010128.
