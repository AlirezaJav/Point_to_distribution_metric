/*
 * Software License Agreement
 *
 *  Point to plane metric for point cloud distortion measurement
 *  Copyright (c) 2017, MERL
 *
 *  All rights reserved.
 *
 *  Contributors:
 *    Dong Tian <tian@merl.com>
 *    Alireza Javaheri <alireza.javaheri@lx.it.pt>   -> Point-to-Distribution Metric is added
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <iostream>
#include <sstream>
#include "pcc_processing.hpp"
#include "pcc_distortion.hpp"
#include "clockcom.hpp"

#ifdef OPENMP_FOUND
#include <omp.h>
#endif

#include "program-options-lite/program_options_lite.h"
using namespace std;
using namespace pcc_quality;
using namespace pcc_processing;
namespace po = df::program_options_lite;
void printUsage( //int ac, char * av[],
    po::Options& opts
    ) {
  cout << endl;
  cout << "Usage: pc_error --fileA=infileA --fileB=infileB [options] " << endl;
  cout << endl;
  cout << "Options: " << endl;
  doHelp( std::cout, opts, 78 );
  cout << endl;

  cout << "Example: " << endl;
  cout << "   ./test/pc_error \\" << endl;
  cout << "          --fileA=./loot/loot_vox10_1000.ply \\" << endl;
  cout << "          --fileB=./S23C2AIR01_loot_dec_1000.ply \\" << endl;
  cout << "          --inputNorm=./loot/loot_vox10_1000_n.ply \\" << endl;
  cout << "          --color=1 \\" << endl;
  cout << "          --resolution=1023 " << endl;
  cout << endl;


  cout << "  #####################################################################" << endl;
  cout << "  #                                                                   #" << endl;
  cout << "  #  The parsing options process have been updated to uniformize      #" << endl;
  cout << "  #  the PCC softwares and use: dependencies/program-options-lite.    #" << endl;
  cout << "  #  This library defined a parsing process a little different than   #" << endl;
  cout << "  #  the Boost library, previously used.                              #" << endl;
  cout << "  #                                                                   #" << endl;
  cout << "  #  The command line options must be updated and:                    #" << endl;
  cout << "  #    * the short options without parameters must be                 #" << endl;
  cout << "  #      updated and now take an argument: ( \"-c\" => \"-c 1\")          #" << endl;
  cout << "  #    * the long options are required to use the \"--option=value\"    #" << endl;
  cout << "  #      form, rather than the previous \"--option value\" form.        #" << endl;
  cout << "  #                                                                   #" << endl;
  cout << "  #####################################################################" << endl;

}

int parseCommand( int ac, char * av[], commandPar &cPar )
{
  bool print_help = ac == 1;
  po::Options opts;
  po::ErrorReporter err;
  try {
    opts.addOptions()
       ("help",           print_help,           false,      "This help text")

       ("a,fileA",        cPar.file1,           string(""), "Input file 1, original version" )
       ("b,fileB",        cPar.file2,           string(""), "Input file 2, processed version" )
       ("n,inputNorm",    cPar.normIn,          string(""), "File name to import the normals of original point "
                                                            "cloud, if different from original file 1n" )

       ("s,singlePass",   cPar.singlePass,      false,      "Force running a single pass, where the loop "
                                                            "is over the original point cloud" )
															
	   ("k,distSize",     cPar.mhdKNN,          31.0f,        "Size of the distribution for mahalanobis distance" )
													
       ("d,hausdorff",    cPar.hausdorff,       false,      "Send the Haursdorff metric as well" )
       ("c,color",        cPar.bColor,          false,      "Check color distortion as well" )
       ("l,lidar",        cPar.bLidar,          false,      "Check lidar reflectance as well" )
       ("r,resolution",   cPar.resolution,      0.0f,       "Specify the intrinsic resolution" )
       ("dropdups",       cPar.dropDuplicates,  2,          "0(detect), 1(drop), 2(average) subsequent points "
                                                            "with same coordinates" )
       ("neighborsProc",  cPar.neighborsProc,   1,          "0(undefined), 1(average), 2(weighted average), "
                                                            "3(min), 4(max) neighbors with same geometric distance" )
       ("averageNormals", cPar.bAverageNormals, true,       "0(undefined), 1(average normal based on neighbors "
                                                            "with same geometric distance)" )
       ("mseSpace",       cPar.mseSpace,        1,          "Colour space used for PSNR calculation\n"
                                                            "0: none (identity) 1: ITU-R BT.709 8: YCgCo-R")
       ("nbThreads",      cPar.nbThreads,       1,          "Number of threads used for parallel processing" );

    setDefaults(opts);
    const list<const char *> &argv_unhandled = scanArgv(opts, ac, (const char **)av, err);
    if( err.is_errored   ) { print_help = true; };
    for (const auto arg : argv_unhandled) {
      err.warn() << "Unhandled argument ignored: " << arg << "\n";
      print_help = true;
    }
    if( cPar.file1 == "" ) { err.error() << "File 1 parameters not correct \n"; print_help = true; }
    if( cPar.file2 == "" ) { err.error() << "File 2 parameters not correct \n"; print_help = true; }
  }
  catch(std::exception& e) {
    cout << e.what() << "\n";
    printUsage( opts );
  }
  if( print_help ) {
    printUsage( opts );
    return false;
  }
  // Check whether your system is compatible with my assumptions
  int szFloat  = sizeof(float)*8;
  int szDouble = sizeof(double)*8;
  int szShort  = sizeof(short)*8;
  int szInt    = sizeof(int)*8;
  // int szLong = sizeof(long long)*8;

  if ( szFloat != 32 || szDouble != 64 || szShort != 16 || szInt != 32 ) //  || szLong != 64
  {
    cout << "Warning: Your system is incompatible with our assumptions below: " << endl;
    cout << "float: "  << sizeof( float  )*8 << endl;
    cout << "double: " << sizeof( double )*8 << endl;
    cout << "short: "  << sizeof( short  )*8 << endl;
    cout << "int: "    << sizeof( int    )*8 << endl;
    // cout << "long long: "<< sizeof(long long)*8 << endl;
    cout << endl;
    return 0;
  }
  return 1;
  // Confict check

}

void printCommand( commandPar &cPar )
{
  cout << "infile1:        " << cPar.file1            << endl;
  cout << "infile2:        " << cPar.file2            << endl;
  cout << "normal1:        " << cPar.normIn           << endl;
  cout << "singlePass:     " << cPar.singlePass       << endl;
  cout << "distSize:       " << cPar.mhdKNN           << endl;
  cout << "hausdorff:      " << cPar.hausdorff        << endl;
  cout << "color:          " << cPar.bColor           << endl;
  cout << "lidar:          " << cPar.bLidar           << endl;
  cout << "resolution:     " << cPar.resolution       << endl;
  cout << "dropDuplicates: " << cPar.dropDuplicates   << endl;
  cout << "neighborsProc:  " << cPar.neighborsProc    << endl;
  cout << "averageNormals: " << cPar.bAverageNormals  << endl;
  cout << "mseSpace:       " << cPar.mseSpace         << endl;
  cout << "nbThreads:      " << cPar.nbThreads        << endl;
  if (cPar.singlePass) {
    cout << "force running a single pass" << endl;
  }
  cout << endl;
}

int main (int argc, char *argv[])
{
  // Print the version information
  cout << "PCC quality measurement software, version " << PCC_QUALITY_VERSION << endl << endl;

  commandPar cPar;
  if ( parseCommand( argc, argv, cPar ) == 0 ) {
    return 0;
  }

  printCommand( cPar );

#ifdef OPENMP_FOUND
  if( cPar.nbThreads != 0 )
  {
    omp_set_dynamic(0);                  // Explicitly disable dynamic teams
    omp_set_num_threads(cPar.nbThreads); // nb threads for all consecutive parallel regions
  }
#else
  printf("Warning: OpenMP is not found, multi-threading is disabled. \n");
#endif
  PccPointCloud inCloud1;
  PccPointCloud inCloud2;
  PccPointCloud inNormal1;

  // Normals may come from either a seperate file, or from file1(a).
  PccPointCloud* pNormal1 = &inCloud1;

  if (inCloud1.load(cPar.file1, false, cPar.dropDuplicates, cPar.neighborsProc))
  {
    cout << "Error reading reference point cloud:" << cPar.file1 << endl;
    return -1;
  }
  cout << "Reading file 1 done." << endl;

  // NB: no need to load the normals again if they will be loaded from a
  //     previously loaded file.
  if (cPar.file1 == cPar.normIn && inCloud1.bNormal)
  {
    cout << "Normals loaded from file1." << endl;
  }
  else if (cPar.normIn != "")
  {
    if (inNormal1.load(cPar.normIn, true, cPar.dropDuplicates))
    {
      cout << "Error reading normal reference point cloud:" << cPar.normIn << endl;
      return -1;
    }
    cout << "Reading normal 1 done." << endl;
    pNormal1 = &inNormal1;
  }

  // If no normals are available, can't do plane2plane
  cPar.c2c_only = !pNormal1->bNormal;

  if (cPar.file2 != "")
  {
    if (inCloud2.load(cPar.file2, false, cPar.dropDuplicates, cPar.neighborsProc))
    {
      cout << "Error reading the second point cloud: " << cPar.file2 << endl;
      return -1;
    }
    cout << "Reading file 2 done." << endl;
  }

  // compute the point to distribution distances, as well as point to point & point to plane distances
  const int t0 = GetTickCount();
  qMetric qm;
  computeQualityMetric(inCloud1, *pNormal1, inCloud2, cPar, qm);

  const int t1 = GetTickCount();
  cout << "Job done! " << (t1 - t0) * 1e-3 << " seconds elapsed (excluding the time to load the point clouds)." << endl;
  return 0;
}
