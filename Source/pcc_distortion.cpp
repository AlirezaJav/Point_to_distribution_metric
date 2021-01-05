/*
 * Software License Agreement
 *
 *  Point to plane metric for point cloud distortion measurement
 *  Copyright (c) 2016, MERL
 *
 *  All rights reserved.
 *
 *  Contributors:
 *    Dong Tian <tian@merl.com>
 *    Maja Krivokuca <majakri01@gmail.com>
 *    Phil Chou <philchou@msn.com>
 *    Alireza Javaheri <alireza.javaheri@lx.it.pt>
 *
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

#include <cstdint>
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits.h>

#include <Eigen\Dense>
#include <math.h> 
#include <numeric>

#include "pcc_processing.hpp"
#include "pcc_distortion.hpp"

using namespace std;
using namespace pcc_quality;
using namespace nanoflann;

// Representation of linear point cloud indexes in kd-tree
typedef uint32_t index_type;

// Representation of distance metric during kd-tree search
typedef double distance_type;

// A version of nanoflann::metric_L2 that forces metric (distance)
// calculations to be performed using double precision floats.
// NB: by default nanoflann::metric_L2 will be used with the metric
//     type of T = num_t (the coordinate type).
struct metric_L2_double {
  template<class T, class DataSource>
  struct traits {
    typedef nanoflann::L2_Adaptor<T,DataSource,double> distance_t;
  };
};

typedef KDTreeVectorOfVectorsAdaptor<
    vector<PointXYZSet::point_type>,     // array type
    PointXYZSet::point_type::value_type, // coordinate type
    3,                                   // num dimensions
    metric_L2_double,                    // distance class
    index_type                           // index type (eg size_t)
    > my_kd_tree_t;

#define PRINT_TIMING 0

/*
 ***********************************************
   Implementation of local functions
 ***********************************************
 */

 /**!
 * \function
 *   Compute the outer product of two vectors
 * \parameters
 *   @param vect1: first vector
 *   @param vect1: second vector
 *   @param mat: output matrix
 * \author
 *   Alireza Javaheri, IT (alireza.javaheri@lx.it.pt)
 */
void outer_product(vector<double> vec1, vector<double> vec2, vector<vector<double>>& mat) {
	for (int i = 0; i < vec1.size(); i++) {
		for (int j = 0; j < vec2.size(); j++) {
			mat[i][j] = vec1[i] * vec2[j];
		}
	}
}
 
 /**!
 * \function
 *   Compute the mean of a point set, later used
     to find the distance
 * \parameters
 *   @param cloud: point cloud
 *   @param indices: indices of point set in point cloud
 *   @param population: size of point set 
 * \author
 *   Alireza Javaheri, IT (alireza.javaheri@lx.it.pt)
 */
vector<double> findMean(PccPointCloud &cloud, index_type *indices, int population)
{
	vector<double> mean(3);
	double sumX = 0;
	double sumY = 0;
	double sumZ = 0;
	for (unsigned  i = 0; i < population; i++)
	{
		sumX += cloud.xyz.p[indices[i]][0];
		sumY += cloud.xyz.p[indices[i]][1];
		sumZ += cloud.xyz.p[indices[i]][2];
	}
	mean[0] = sumX / population;
	mean[1] = sumY / population;
	mean[2] = sumZ / population;
	return mean;
}
 
/**!
* \function
*   Compute the covariance matrix of a point set, later used
to find the distance
* \parameters
*   @param cloud: point cloud
*   @param indices: indices of point set in point cloud
*   @param population: size of point set
*   @param mean: mean of the population
* \author
*   Alireza Javaheri, IT (alireza.javaheri@lx.it.pt)
*/
Eigen::Matrix3d findCovMat(PccPointCloud &cloud, index_type *indices, int population, vector<double> mean)
{
	vector<double> normD(3);
	vector<vector<double>> C_iter(3,vector<double>(3));
	vector<vector<double>> C_sum = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };
	//vector<vector<double>> C_final;
	Eigen::MatrixXd C_final(3, 3);

	for (int i = 0; i < population; i++)
	{
		normD[0] = cloud.xyz.p[indices[i]][0] - mean[0];
		normD[1] = cloud.xyz.p[indices[i]][1] - mean[1];
		normD[2] = cloud.xyz.p[indices[i]][2] - mean[2];
		outer_product(normD, normD, C_iter);
		for (int j = 0; j < normD.size(); j++)
		{
			for (int k = 0; k < normD.size(); k++)
			{
				C_sum[j][k] = C_sum[j][k] + C_iter[j][k];
			}
		}
	}
	for (int j = 0; j < normD.size(); j++)
	{
		for (int k = 0; k < normD.size(); k++)
		{
			//C_final[j][k] = C_sum[j][k] / population;
			C_final(j, k) = C_sum[j][k] / population;
		}
	}

	return C_final;
}

/**!
* \function
*   Compute the covariance matrix of a point set, later used
to find the distance
* \parameters
*   @param refPoint: reference point in PC A
*   @param mean: mean of the nearest neighbor population
*   @param cov: covariance matrix of the nearest neighbor population
* \author
*   Alireza Javaheri, IT (alireza.javaheri@lx.it.pt)
*/
double mahalanobisDist(vector<double> refPoint, vector<double> mean, Eigen::Matrix3d cov)
{
	double MD;
	Eigen::Matrix3d invCov;
	//Eigen::Matrix3d invCovD;
	Eigen::Vector3d ref;
	Eigen::Matrix<double, 1, 3> temp;
	Eigen::Matrix3d covD;

	ref << refPoint[0] - mean[0], refPoint[1] - mean[1], refPoint[2] - mean[2];
	Eigen::LLT<Eigen::MatrixXd> lltOfA(cov); // compute the Cholesky decomposition of A

    /*
		for voxelized point cloud, becasue largest eigenvalue is always a large number, determinant works ...
		if floating point data is used, threshold should be defined on condition number of covariance matrix
		Threshold on condition number of covariance matrix is not tested!
	*/
	if (cov.determinant() < 0.000001 || lltOfA.info() == Eigen::NumericalIssue)   
	{                                                                             
		return NAN;
	}
	else
	{
		invCov = cov.inverse();
		temp = ref.transpose() * invCov;
		double x = temp * ref;
		if (x < 0)
		{
			int test = 0;  // for test purposes!
		}
		MD = sqrt(x);
	}

	return MD;
}
 
/**!
 * \function
 *   Compute the minimum and maximum NN distances, find out the
 *   intrinsic resolutions
 * \parameters
 *   @param cloudA: point cloud
 *   @param minDist: output
 *   @param maxDist: output
 * \note
 *   PointT typename of point used in point cloud
 * \author
 *   Dong Tian, MERL
 */
void
findNNdistances(PccPointCloud &cloudA, double &minDist, double &maxDist)
{
  maxDist =  numeric_limits<double>::min();
  minDist =  numeric_limits<double>::max();
  double distTmp = 0;
  mutex myMutex;

  my_kd_tree_t mat_index(3, cloudA.xyz.p, 10); // dim, cloud, max leaf

#pragma omp parallel for
  for (long i = 0; i < cloudA.size; ++i)
  {
    // cout << "*** " << i << endl;
    // do a knn search
    const size_t num_results = 3;
    std::array<index_type,num_results> indices;
    std::array<distance_type,num_results> sqrDist;

    mat_index.query(&cloudA.xyz.p[i][0], num_results, &indices[0], &sqrDist[0]);

    if (indices[0] != i || sqrDist[1] <= 0.0000000001)
    {
      // Print some warnings
      // cerr << "Error! nFound = " << nFound << ", i, iFound = " << i << ", " << indices[0] << ", " << indices[1] << endl;
      // cerr << "       Distances = " << sqrDist[0] << ", " << sqrDist[1] << endl;
      // cerr << "  Some points are repeated!" << endl;
    }
    else
    {
      // Use the second one. assume the first one is the current point
      myMutex.lock();
      distTmp = sqrt( sqrDist[1] );
      if (distTmp > maxDist)
        maxDist = distTmp;
      if (distTmp < minDist)
        minDist = distTmp;
      myMutex.unlock();
    }
  }
}

/**!
 * \function
 *   Convert the MSE error to PSNR numbers
 * \parameters
 *   @param cloudA:  the original point cloud
 *   @param dist2: the sqr of the distortion
 *   @param p: the peak value for conversion
 *   @param factor: default 1.0. For geometry errors value 3.0 should be provided
 * \return
 *   psnr value
 * \note
 *   PointT typename of point used in point cloud
 * \author
 *   Dong Tian, MERL
 */
float
getPSNR(float dist2, float p, float factor = 1.0)
{
  float max_energy = p * p;
  float psnr = 10 * log10( (factor * max_energy) / dist2 );

  return psnr;
}

/**!
 * \function
 *   Derive the normals for the decoded point cloud based on the
 *   normals in the original point cloud
 * \parameters
 *   @param cloudA:  the original point cloud
 *   @param cloudNormalsA: the normals in the original point cloud
 *   @param cloudB:  the decoded point cloud
 *   @param cloudNormalsB: the normals in the original point
 *     cloud. Output parameter
 * \note
 *   PointT typename of point used in point cloud
 * \author
 *   Dong Tian, MERL
 */
void
scaleNormals(PccPointCloud &cloudNormalsA, PccPointCloud &cloudB, PccPointCloud &cloudNormalsB, bool bAverageNormals)
{
  // Prepare the buffer to compute the average normals
#if PRINT_TIMING
  clock_t t1 = clock();
#endif

  cloudNormalsB.normal.init(cloudB.size);
  vector<int> counts(cloudB.size);

  if (1) {
    my_kd_tree_t mat_indexB(3, cloudB.xyz.p, 10); // dim, cloud, max leaf
    for (long i = 0; i < cloudNormalsA.size; i++)
    {
      if( bAverageNormals ) {
        const size_t num_results_max = 30;
        const size_t num_results_incr = 5;
        size_t num_results = 0;

        std::array<index_type,num_results_max> indices;
        std::array<distance_type,num_results_max> sqrDist;
        do {
          num_results  += num_results_incr;
          mat_indexB.query(&cloudNormalsA.xyz.p[i][0], num_results, &indices[0], &sqrDist[0]);
        } while( sqrDist[0] == sqrDist[num_results-1] && num_results + num_results_incr <= num_results_max );
        for( size_t j=0;j<num_results;j++){
          if( sqrDist[0] == sqrDist[j] ) {
            cloudNormalsB.normal.n[indices[j]][0] += cloudNormalsA.normal.n[i][0];
            cloudNormalsB.normal.n[indices[j]][1] += cloudNormalsA.normal.n[i][1];
            cloudNormalsB.normal.n[indices[j]][2] += cloudNormalsA.normal.n[i][2];
            counts[indices[j]]++;
          }
        }
      }
      else
      {
        const size_t num_results = 1;
        std::array<index_type,num_results> indices;
        std::array<distance_type,num_results> sqrDist;

        mat_indexB.query(&cloudNormalsA.xyz.p[i][0], num_results, &indices[0], &sqrDist[0]);

        cloudNormalsB.normal.n[indices[0]][0] += cloudNormalsA.normal.n[i][0];
        cloudNormalsB.normal.n[indices[0]][1] += cloudNormalsA.normal.n[i][1];
        cloudNormalsB.normal.n[indices[0]][2] += cloudNormalsA.normal.n[i][2];
        counts[indices[0]]++;
      }
    }
  }

  // average now
  my_kd_tree_t mat_indexA(3, cloudNormalsA.xyz.p, 10); // dim, cloud, max leaf
  for (long i = 0; i < cloudB.size; i++)
  {
    int nCount = counts[i];
    if (nCount > 0)      // main branch
    {
      cloudNormalsB.normal.n[i][0] = cloudNormalsB.normal.n[i][0] / nCount;
      cloudNormalsB.normal.n[i][1] = cloudNormalsB.normal.n[i][1] / nCount;
      cloudNormalsB.normal.n[i][2] = cloudNormalsB.normal.n[i][2] / nCount;
    }
    else
    {
      if( bAverageNormals ) 
      {
        const size_t num_results_max = 30;
        const size_t num_results_incr = 5;
        size_t num_results = 0;
        std::array<index_type,num_results_max> indices;
        std::array<distance_type,num_results_max> sqrDist;
        do {
          num_results  += num_results_incr;
          mat_indexA.query(&cloudB.xyz.p[i][0], num_results, &indices[0], &sqrDist[0]);
        } while( sqrDist[0] == sqrDist[num_results-1] && num_results + num_results_incr <= num_results_max );
        size_t num = 0;
        for( size_t j=0;j<num_results;j++){
          if( sqrDist[0] == sqrDist[j] ) {
            cloudNormalsB.normal.n[i][0] += cloudNormalsA.normal.n[indices[j]][0];
            cloudNormalsB.normal.n[i][1] += cloudNormalsA.normal.n[indices[j]][1];
            cloudNormalsB.normal.n[i][2] += cloudNormalsA.normal.n[indices[j]][2];
            num++;
          }
        }
        cloudNormalsB.normal.n[i][0] /= num;
        cloudNormalsB.normal.n[i][1] /= num;
        cloudNormalsB.normal.n[i][2] /= num;
      }
      else
      {
        const size_t num_results = 1;
        std::array<index_type,num_results> indices;
        std::array<distance_type,num_results> sqrDist;

        mat_indexA.query(&cloudB.xyz.p[i][0], num_results, &indices[0], &sqrDist[0]);

        cloudNormalsB.normal.n[i][0] = cloudNormalsA.normal.n[indices[0]][0];
        cloudNormalsB.normal.n[i][1] = cloudNormalsA.normal.n[indices[0]][1];
        cloudNormalsB.normal.n[i][2] = cloudNormalsA.normal.n[indices[0]][2];
      }
    }
  }

  // Set the flag
  cloudNormalsB.bNormal = true;

#if PRINT_TIMING
  clock_t t2 = clock();
  cout << "   Converting normal vector DONE. It takes " << (t2-t1)/CLOCKS_PER_SEC << " seconds (in CPU time)." << endl;
#endif
}

/**
   \brief helper function to convert RGB to YUV (BT.709 or YCoCg-R)
 */
void convertRGBtoYUV(int type, const std::array<unsigned char, 3> &in_rgb,
                           float *out_yuv) {
  // color space conversion to YUV

  if (type == 0)
  {
    for (int d = 0; d < 3; d++)
      out_yuv[d] = float(in_rgb[d]);
  }
  else if (type == 8)
  {
    int g = in_rgb[1];
    int b = in_rgb[2];
    int r = in_rgb[0];

    int co = r - b;
    int t = b + (co >> 1);
    int cg = g - t;
    int y = t + (cg >> 1);

    int offset = 1 << 8;

    out_yuv[0] = y;
    out_yuv[1] = co + offset;
    out_yuv[2] = cg + offset;
  }
  else // type 1
  {
    out_yuv[0] = float((0.2126 * in_rgb[0] + 0.7152 * in_rgb[1] + 0.0722 * in_rgb[2]) / 255.0);
    out_yuv[1] = float((-0.1146 * in_rgb[0] - 0.3854 * in_rgb[1] + 0.5000 * in_rgb[2]) / 255.0 + 0.5000);
    out_yuv[2] = float((0.5000 * in_rgb[0] - 0.4542 * in_rgb[1] - 0.0458 * in_rgb[2]) / 255.0 + 0.5000);
  }
}

/**!
 * \function
 *   To compute "one-way" quality metric: Point-to-Point, Point-to-Plane
 *   and RGB. Loop over each point in A. Normals in B to be used
 *
 *   1) For each point in A, find a corresponding point in B.
 *   2) Form an error vector between the point pair.
 *   3) Use the length of the error vector as point-to-point measure
 *   4) Project the error vector along the normals in B, use the length
 *   of the projected error vector as point-to-plane measure
 *
 *   @param cloudA: Reference point cloud. e.g. the original cloud, on
 *     which normals would be estimated. It is the full set of point
 *     cloud. Multiple points in count
 *   @param cloudB: Processed point cloud. e.g. the decoded cloud
 *   @param cPar: Command line parameters
 *   @param cloudNormalsB: Normals for cloudB
 *   @param metric: updated quality metric, to be returned
 * \note
 *   PointT typename of point used in point cloud
 * \author
 *   Dong Tian, MERL
 *
 * \note
 * 		point to distribution quality metric computation is added to the quality metric measurement loop
 *      /author
 *     	 Alireza Javaheri, IT (alireza.javaheri@lx.it.pt)
 */
void
findMetric(PccPointCloud &cloudA, PccPointCloud &cloudB, commandPar &cPar, PccPointCloud &cloudNormalsB, qMetric &metric)
{
  mutex myMutex;

#if PRINT_TIMING
  clock_t t2 = clock();
#endif

  //Point-to-Distribution Variables
  int illCounter = 0;
  long double SumMahDist = 0;
  long double SumSMahDist = 0;
  double MinMahDist = std::numeric_limits<double>::max();
  double MaxMahDist = std::numeric_limits<double>::min();
  double MinSMahDist = std::numeric_limits<double>::max();
  double MaxSMahDist = std::numeric_limits<double>::min();
  double min_dist_b_c2c = std::numeric_limits<double>::max();
  vector<double> AllLostMahErrorVectors(cloudA.size);
  vector<double> AllCorrDistErrorVectors(cloudA.size);
  vector<double> AllLostSMahErrorVectors(cloudA.size);
  vector<double> AllCorrSDistErrorVectors(cloudA.size);

  double max_dist_b_c2c = std::numeric_limits<double>::min();
  double sse_dist_b_c2c = 0;
  double max_dist_b_c2p = std::numeric_limits<double>::min();
  double sse_dist_b_c2p = 0;
  double max_reflectance = std::numeric_limits<double>::min();
  double sse_reflectance = 0;
  long num = 0;

  double sse_color[3];
  sse_color[0] = sse_color[1] = sse_color[2] = 0.0;
  double max_colorRGB[3];
  max_colorRGB[0] = max_colorRGB[1] = max_colorRGB[2] = std::numeric_limits<double>::min();

  my_kd_tree_t mat_indexB(3, cloudB.xyz.p, 10); // dim, cloud, max leaf

  const size_t num_results_max  = 30;
  const size_t num_results_incr = 5;

  // point to distribution
  my_kd_tree_t mat_indexB_P2D(3, cloudB.xyz.p, 10); // dim, cloud, max leaf
  const size_t max_results_p2d = 100;

#if DUPLICATECOLORS_DEBUG
  long NbNeighborsDst[num_results_max] = {};
#endif

#pragma omp parallel for
  for (long i = 0; i < cloudA.size; i++)
  {
    size_t num_results = num_results_incr;
    // For point 'i' in A, find its nearest neighbor in B. store it in 'j'
    std::array<index_type,num_results_max> indices;
    std::array<distance_type,num_results_max> sqrDist;
    do {
      num_results += num_results_incr;
      if (!mat_indexB.query(&cloudA.xyz.p[i][0], num_results, &indices[0], &sqrDist[0]))
      {
        cout << " WARNING: requested neighbors could not be found " << endl;
      }
    } while( sqrDist[0] == sqrDist[num_results-1] && cPar.bAverageNormals && num_results + num_results_incr <= num_results_max );

	//Point to Distribution Distance computation goes here!
	long double md = 0;
	long double smd = 0;
	vector<double> mean(3);
	Eigen::Matrix3d covMat;
	vector<double> refPoint(3);
	vector<double> tarPoint(3);
	std::array<index_type, max_results_p2d> indices_p2d;
	std::array<distance_type, max_results_p2d> sqrDist_p2d;

	if (!mat_indexB_P2D.query(&cloudA.xyz.p[i][0], cPar.mhdKNN, &indices_p2d[0], &sqrDist_p2d[0]))
	{
		cout << " WARNING: requested neighbors could not be found " << endl;
	}

	mean = findMean(cloudB, &indices_p2d[0], cPar.mhdKNN);
	covMat = findCovMat(cloudB, &indices_p2d[0], cPar.mhdKNN, mean);
	refPoint[0] = cloudA.xyz.p[i][0];
	refPoint[1] = cloudA.xyz.p[i][1];
	refPoint[2] = cloudA.xyz.p[i][2];
	
	md = mahalanobisDist(refPoint, mean, covMat);
	
    int j = indices[0];
    if (j < 0)
      continue;

    std::vector<size_t> sameDistPoints;
    if (cPar.bColor || (!cPar.c2c_only && cloudNormalsB.bNormal) ) {
      sameDistPoints.push_back( indices[0] );
      for (size_t n = 1; n < num_results; n++) {
        if (fabs(sqrDist[n] - sqrDist[n - 1]) < 1e-8) {
          sameDistPoints.push_back( indices[n] );
        } else {
          break;
        }
      }
    }

    // Compute the error vector
    std::array<double,3> errVector;
    errVector[0] = cloudA.xyz.p[i][0] - cloudB.xyz.p[j][0];
    errVector[1] = cloudA.xyz.p[i][1] - cloudB.xyz.p[j][1];
    errVector[2] = cloudA.xyz.p[i][2] - cloudB.xyz.p[j][2];

    // Compute point-to-point, which should be equal to sqrt( sqrDist[0] )
    double distProj_c2c = errVector[0] * errVector[0] + errVector[1] * errVector[1] + errVector[2] * errVector[2];

    // Compute point-to-plane
    // Normals in B will be used for point-to-plane
    double distProj = 0.0;
    if (!cPar.c2c_only && cloudNormalsB.bNormal) {
      if( cPar.bAverageNormals ) {
        for ( auto& index : sameDistPoints ) {
          if ( !isnan( cloudNormalsB.normal.n[index][0] ) &&
               !isnan( cloudNormalsB.normal.n[index][1] ) &&
               !isnan( cloudNormalsB.normal.n[index][2] ) ) {
            double dist = pow( ( cloudA.xyz.p[i][0] - cloudB.xyz.p[index][0] ) * cloudNormalsB.normal.n[index][0] +
                               ( cloudA.xyz.p[i][1] - cloudB.xyz.p[index][1] ) * cloudNormalsB.normal.n[index][1] +
                               ( cloudA.xyz.p[i][2] - cloudB.xyz.p[index][2] ) * cloudNormalsB.normal.n[index][2], 2.f );
            distProj += dist;
          } else {
            distProj += cloudA.xyz.p[i][0] - cloudB.xyz.p[index][0] + 
                        cloudA.xyz.p[i][1] - cloudB.xyz.p[index][1] +
                        cloudA.xyz.p[i][2] - cloudB.xyz.p[index][2];
          }
        }
        distProj /= (double)sameDistPoints.size();
      } else {
        if ( !isnan( cloudNormalsB.normal.n[j][0] ) &&
             !isnan( cloudNormalsB.normal.n[j][1] ) &&
             !isnan( cloudNormalsB.normal.n[j][2] ) ) {
          distProj = ( errVector[0] * cloudNormalsB.normal.n[j][0] +
                       errVector[1] * cloudNormalsB.normal.n[j][1] +
                       errVector[2] * cloudNormalsB.normal.n[j][2] );
          distProj *= distProj;  // power 2 for MSE
        } else {
          distProj = distProj_c2c;
        }
      }
    }

    double distColor[3];
    distColor[0] = distColor[1] = distColor[2] = 0.0;
    double distColorRGB[3];
    distColorRGB[0] = distColorRGB[1] = distColorRGB[2] = std::numeric_limits<double>::min();
    if (cPar.bColor && cloudA.bRgb && cloudB.bRgb)
    {
      float out[3];
      float in[3];
      convertRGBtoYUV(cPar.mseSpace, cloudA.rgb.c[i], in);

      if (cPar.neighborsProc)
      {
        unsigned int r = 0, g = 0, b = 0;
        std::array<unsigned char,3> color;
        switch (cPar.neighborsProc)
        {
          case 0:
            break;
          case 1:     // Average
          case 2:     // Weighted average
          {
            int nbdupcumul = 0;
            for ( auto& index : sameDistPoints ) {
              int nbdup = cloudB.xyz.nbdup[ index ];
              r += nbdup * cloudB.rgb.c[index][0];
              g += nbdup * cloudB.rgb.c[index][1];
              b += nbdup * cloudB.rgb.c[index][2];
              nbdupcumul += nbdup;
            }
            assert(nbdupcumul);
            color[0] = (unsigned char)round((double)r / nbdupcumul);
            color[1] = (unsigned char)round((double)g / nbdupcumul);
            color[2] = (unsigned char)round((double)b / nbdupcumul);
          }
          break;
          case 3:   // Min
          {
            unsigned int distColorMin = (std::numeric_limits<unsigned int>::max)();
            size_t indexMin = 0;
            for ( auto& index : sameDistPoints ) {
              unsigned int distRGB = (cloudA.rgb.c[i][0] - cloudB.rgb.c[index][0]) * (cloudA.rgb.c[i][0] - cloudB.rgb.c[index][0])
                                   + (cloudA.rgb.c[i][1] - cloudB.rgb.c[index][1]) * (cloudA.rgb.c[i][1] - cloudB.rgb.c[index][1])
                                   + (cloudA.rgb.c[i][2] - cloudB.rgb.c[index][2]) * (cloudA.rgb.c[i][2] - cloudB.rgb.c[index][2]);
              if ( distRGB < distColorMin) {
                distColorMin = distRGB;
                indexMin = index;
              }
            }
            color = cloudB.rgb.c[indexMin];
          }
          break;
          case 4:   // Max
          {
            unsigned int distColorMax = 0;
            size_t indexMax = 0;
            for ( auto& index : sameDistPoints ) {
              unsigned int distRGB = (cloudA.rgb.c[i][0] - cloudB.rgb.c[index][0]) * (cloudA.rgb.c[i][0] - cloudB.rgb.c[index][0])
                                   + (cloudA.rgb.c[i][1] - cloudB.rgb.c[index][1]) * (cloudA.rgb.c[i][1] - cloudB.rgb.c[index][1])
                                   + (cloudA.rgb.c[i][2] - cloudB.rgb.c[index][2]) * (cloudA.rgb.c[i][2] - cloudB.rgb.c[index][2]);
              if (distRGB > distColorMax) {
                distColorMax = distRGB;
                indexMax = index;
              }
            }
            color = cloudB.rgb.c[indexMax];
          }
          break;
        }

        convertRGBtoYUV(cPar.mseSpace, color, out);
        distColorRGB[0] = (cloudA.rgb.c[i][0] - color[0]) * (cloudA.rgb.c[i][0] - color[0]);
        distColorRGB[1] = (cloudA.rgb.c[i][1] - color[1]) * (cloudA.rgb.c[i][1] - color[1]);
        distColorRGB[2] = (cloudA.rgb.c[i][2] - color[2]) * (cloudA.rgb.c[i][2] - color[2]);
      }
      else
      {
        convertRGBtoYUV(cPar.mseSpace, cloudB.rgb.c[j], out);
        distColorRGB[0] = (cloudA.rgb.c[i][0] - cloudB.rgb.c[j][0]) * (cloudA.rgb.c[i][0] - cloudB.rgb.c[j][0]);
        distColorRGB[1] = (cloudA.rgb.c[i][1] - cloudB.rgb.c[j][1]) * (cloudA.rgb.c[i][1] - cloudB.rgb.c[j][1]);
        distColorRGB[2] = (cloudA.rgb.c[i][2] - cloudB.rgb.c[j][2]) * (cloudA.rgb.c[i][2] - cloudB.rgb.c[j][2]);
      }

      distColor[0] = (in[0] - out[0]) * (in[0] - out[0]);
      distColor[1] = (in[1] - out[1]) * (in[1] - out[1]);
      distColor[2] = (in[2] - out[2]) * (in[2] - out[2]);
    }

    double distReflectance;
    distReflectance = 0.0;
    if (cPar.bLidar && cloudA.bLidar && cloudB.bLidar)
    {
      double diff = cloudA.lidar.reflectance[i] - cloudB.lidar.reflectance[j];
      distReflectance = diff * diff;
    }

    myMutex.lock();

    num++;
    // mean square distance
    sse_dist_b_c2c += distProj_c2c;
    if (distProj_c2c > max_dist_b_c2c)
      max_dist_b_c2c = distProj_c2c;
    if (!cPar.c2c_only)
    {
      sse_dist_b_c2p += distProj;
      if (distProj > max_dist_b_c2p)
        max_dist_b_c2p = distProj;
    }
	
	// also find minimum point to point distance
	if (distProj_c2c < min_dist_b_c2c)
		min_dist_b_c2c = distProj_c2c;
	
	// Check and record ill-conditioned covariance matrices
	if (!isnan(md))   
	{
		if (md > MaxMahDist)
			MaxMahDist = md;
		if (md < MinMahDist)
			MinMahDist = md;
		SumMahDist += md;

		AllLostMahErrorVectors[i] = md;
		AllCorrDistErrorVectors[i] = -1;

		smd = md * md;	// the square mahalanobis distance
		if (smd > MaxSMahDist)
			MaxSMahDist = smd;
		if (smd < MinSMahDist)
			MinSMahDist = smd;
		SumSMahDist += smd;

		AllLostSMahErrorVectors[i] = smd;
		AllCorrSDistErrorVectors[i] = -1;
	}
	else
	{
		illCounter++;
		//comment below lines to ignore points with an ill-conditioned associated covariance matrix! uncomment to compensate!

		AllLostMahErrorVectors[i] = -1;
		AllCorrDistErrorVectors[i] = sqrt(distProj_c2c);
		AllLostSMahErrorVectors[i] = -1;
		AllCorrSDistErrorVectors[i] = distProj_c2c;
	}
	
	
    if (cPar.bColor)
    {
      sse_color[0] += distColor[0];
      sse_color[1] += distColor[1];
      sse_color[2] += distColor[2];

      max_colorRGB[0] = max(max_colorRGB[0], distColorRGB[0]);
      max_colorRGB[1] = max(max_colorRGB[1], distColorRGB[1]);
      max_colorRGB[2] = max(max_colorRGB[2], distColorRGB[2]);
    }
    if (cPar.bLidar && cloudA.bLidar && cloudB.bLidar)
    {
      sse_reflectance += distReflectance;
      max_reflectance = max(max_reflectance, distReflectance);
    }

#if DUPLICATECOLORS_DEBUG
    for (long n = 0; n < num_results; n++)
      if (rgb[n].size())
        NbNeighborsDst[n]++;
#endif

    myMutex.unlock();
  }
#if DUPLICATECOLORS_DEBUG
  cout << " DEBUG: " << NbNeighborsDst[1] << " points (" << (float)NbNeighborsDst[1] * 100.0 / cloudA.size << "%) found with at least 2 neighbors at the same minimum distance" << endl;
#endif

  // report ill-conditioned cases
  cout << "\nNumber of points that are compensated or skipped: " << illCounter << endl;
  cout << "Percentage of points that are compensated or skipped: " << double(illCounter) * 100 / cloudA.size << endl<<endl;

  metric.c2p_mse = float( sse_dist_b_c2p / num );
  metric.c2c_mse = float( sse_dist_b_c2c / num );
  metric.c2p_hausdorff = float( max_dist_b_c2p );
  metric.c2c_hausdorff = float( max_dist_b_c2c );
  
  double sumIllError = 0;
  double sumIllSqError = 0;
  if (illCounter)
  {
	  // normalize mahlanobis distances in range of point-to-point
	  for (int i = 0; i < AllLostMahErrorVectors.size(); i++)
	  {
		  if (AllLostMahErrorVectors[i] == -1)
		  {
			  //Normlaize in range of MAhalanobis
			  sumIllError += (((AllCorrDistErrorVectors[i] - sqrt(min_dist_b_c2c)) / (sqrt(max_dist_b_c2c) - sqrt(min_dist_b_c2c))) * (MaxMahDist - MinMahDist)) + MinMahDist;  
			  sumIllSqError += (((AllCorrSDistErrorVectors[i] - min_dist_b_c2c) / (max_dist_b_c2c - min_dist_b_c2c)) * (MaxSMahDist - MinSMahDist)) + MinSMahDist;
		  
			  //P2Po without Nomrmalization
			  /*sumIllError += AllCorrDistErrorVectors[i];*/
		  
		  }
	  }
  }
  SumMahDist += sumIllError;
  metric.mmd = double(SumMahDist / num);

  SumSMahDist += sumIllSqError;
  metric.msmd = double(SumSMahDist / num);

  // from distance to PSNR. cloudA always the original
  metric.c2c_psnr = getPSNR( metric.c2c_mse, metric.pPSNR, 3 );
  metric.c2p_psnr = getPSNR( metric.c2p_mse, metric.pPSNR, 3 );
  metric.c2c_hausdorff_psnr = getPSNR( metric.c2c_hausdorff, metric.pPSNR, 3 );
  metric.c2p_hausdorff_psnr = getPSNR( metric.c2p_hausdorff, metric.pPSNR, 3 );

  metric.mmd_psnr = getPSNR(metric.mmd * metric.mmd, metric.pPSNR, 3);
  metric.msmd_psnr = getPSNR(metric.msmd, metric.pPSNR, 3);
  
  if (cPar.bColor)
  {
    metric.color_mse[0] = float( sse_color[0] / num );
    metric.color_mse[1] = float( sse_color[1] / num );
    metric.color_mse[2] = float( sse_color[2] / num );

    if (cPar.mseSpace == 1) //YCbCr
    {
      metric.color_psnr[0] = getPSNR(metric.color_mse[0], 1.0);
      metric.color_psnr[1] = getPSNR(metric.color_mse[1], 1.0);
      metric.color_psnr[2] = getPSNR(metric.color_mse[2], 1.0);
    }
    else if (cPar.mseSpace == 0) //RGB
    {
      metric.color_psnr[0] = getPSNR(metric.color_mse[0], 255);
      metric.color_psnr[1] = getPSNR(metric.color_mse[1], 255);
      metric.color_psnr[2] = getPSNR(metric.color_mse[2], 255);
    }
    else if (cPar.mseSpace == 8) // YCoCg-R
    {
      metric.color_psnr[0] = getPSNR(metric.color_mse[0], 255);
      metric.color_psnr[1] = getPSNR(metric.color_mse[1], 511);
      metric.color_psnr[2] = getPSNR(metric.color_mse[2], 511);
    }

    metric.color_rgb_hausdorff[0] = float( max_colorRGB[0] );
    metric.color_rgb_hausdorff[1] = float( max_colorRGB[1] );
    metric.color_rgb_hausdorff[2] = float( max_colorRGB[2] );

    metric.color_rgb_hausdorff_psnr[0] = getPSNR( metric.color_rgb_hausdorff[0], 255.0 );
    metric.color_rgb_hausdorff_psnr[1] = getPSNR( metric.color_rgb_hausdorff[1], 255.0 );
    metric.color_rgb_hausdorff_psnr[2] = getPSNR( metric.color_rgb_hausdorff[2], 255.0 );
  }

  if (cPar.bLidar)
  {
    metric.reflectance_mse = float( sse_reflectance / num );
    metric.reflectance_psnr = getPSNR( float( metric.reflectance_mse ), float( std::numeric_limits<unsigned short>::max() ) );
    metric.reflectance_hausdorff = float( max_reflectance );
    metric.reflectance_hausdorff_psnr = getPSNR(metric.reflectance_hausdorff, std::numeric_limits<unsigned short>::max() );
  }

#if PRINT_TIMING
  clock_t t3 = clock();
  cout << "   Error computing takes " << (t3-t2)/CLOCKS_PER_SEC << " seconds (in CPU time)." << endl;
#endif
}

/*
 ***********************************************
   Implementation of exposed functions and classes
 ***********************************************
 */

/**!
 * **************************************
 *  Class commandPar
 *
 *  Dong Tian <tian@merl.com>
 * **************************************
 */

commandPar::commandPar()
{
  file1 = ""; file2 = "";
  normIn = "";
  singlePass = false;
  hausdorff = false;
  c2c_only = false;
  bColor = false;
  bLidar = false;

  //point-to-distribution
  mhdKNN = 31;
  
  resolution = 0.0;
  dropDuplicates = 0;
  neighborsProc = 0;
}

/**!
 * **************************************
 *  Class qMetric
 *
 *  Dong Tian <tian@merl.com>
 * **************************************
 */

qMetric::qMetric()
{
  //point-to-distribution
  mmd = 0.0;
  msmd = 0.0;
  mmd_psnr = 0;
  msmd_psnr = 0;
	
  c2c_mse = 0; c2c_hausdorff = 0;
  c2p_mse = 0; c2p_hausdorff = 0;

  color_mse[0] = color_mse[1] = color_mse[2] = 0.0;
  color_psnr[0] = color_psnr[1] = color_psnr[2] = 0.0;

  color_rgb_hausdorff[0] = color_rgb_hausdorff[1] = color_rgb_hausdorff[2] = 0.0;
  color_rgb_hausdorff_psnr[0] = color_rgb_hausdorff_psnr[1] = color_rgb_hausdorff_psnr[2] = 0.0;

  reflectance_hausdorff = 0.0;
  reflectance_hausdorff_psnr = 0.0;
}

/**!
 * **************************************
 *  Function computeQualityMetric
 *
 *  Dong Tian <tian@merl.com>
 * **************************************
 */

/**!
 * function to compute the symmetric quality metric: Point-to-Point and Point-to-Plane
 *   @param cloudA: point cloud, original version
 *   @param cloudNormalA: point cloud normals, original version
 *   @param cloudB: point cloud, decoded/reconstructed version
 *   @param cPar: input parameters
 *   @param qual_metric: quality metric, to be returned
 *
 * \author
 *   Dong Tian, MERL
 */
void
pcc_quality::computeQualityMetric(PccPointCloud &cloudA, PccPointCloud &cloudNormalsA, PccPointCloud &cloudB, commandPar &cPar, qMetric &qual_metric)
{
  float pPSNR;

  if (cPar.resolution != 0.0)
  {
    cout << "Imported intrinsic resoluiton: " << cPar.resolution << endl;
    pPSNR = cPar.resolution;
  }
  else                          // Compute the peak value on the fly
  {
    double minDist;
    double maxDist;
    findNNdistances(cloudA, minDist, maxDist);
    pPSNR = float( maxDist );
    cout << "Minimum and maximum NN distances (intrinsic resolutions): " << minDist << ", " << maxDist << endl;
  }

  cout << "Peak distance for PSNR: " << pPSNR << endl;
  qual_metric.pPSNR = pPSNR;

  if (cPar.file2 != "")
  {
    // Check cloud size
    size_t orgSize = cloudA.size;
    size_t newSize = cloudB.size;
    float ratio = float(1.0) * newSize / orgSize;
    cout << "Point cloud sizes for org version, dec version, and the scaling ratio: " << orgSize << ", " << newSize << ", " << ratio << endl;
  }

  if (cPar.file2 == "" ) // If no file2 provided, return just after checking the NN
    return;

  // Based on normals on original point cloud, derive normals on reconstructed point cloud
  PccPointCloud cloudNormalsB;
  if (!cPar.c2c_only)
    scaleNormals( cloudNormalsA, cloudB, cloudNormalsB, cPar.bAverageNormals );
  cout << "Normals prepared." << endl;
  cout << endl;

  if (cPar.bColor && (!cloudA.bRgb || !cloudB.bRgb))
  {
    cout << "WARNING: no color properties in input files, disabling color metrics.\n";
    cPar.bColor = false;
  }

  if (cPar.bLidar && (!cloudA.bLidar || !cloudB.bLidar))
  {
    cout << "WARNING: no reflectance property in input files, disabling reflectance metrics.\n";
    cPar.bLidar = false;
  }

  if (cPar.bLidar && cPar.neighborsProc)
  {
    cout << "WARNING: reflectance metrics are computed without neighborsProc parameter.\n";
  }

  // Use "a" as reference
  cout << "1. Use infile1 (A) as reference, loop over A, use normals on B. (A->B).\n";
  qMetric metricA;
  metricA.pPSNR = pPSNR;
  findMetric( cloudA, cloudB, cPar, cloudNormalsB, metricA );

  cout << "   mse1      (p2point): " << metricA.c2c_mse << endl;
  cout << "   mse1,PSNR (p2point): " << metricA.c2c_psnr << endl;
  cout << "   mmd       (p2distr): " << metricA.mmd << endl;
  cout << "   mmd,PSNR  (p2distr): " << metricA.mmd_psnr << endl;
  cout << "   msmd      (p2distr): " << metricA.msmd << endl;
  cout << "   msmd,PSNR (p2distr): " << metricA.msmd_psnr << endl;
  
  if (!cPar.c2c_only)
  {
    cout << "   mse1      (p2plane): " << metricA.c2p_mse << endl;
    cout << "   mse1,PSNR (p2plane): " << metricA.c2p_psnr << endl;
  }
  if ( cPar.hausdorff )
  {
    cout << "   h.       1(p2point): " << metricA.c2c_hausdorff << endl;
    cout << "   h.,PSNR  1(p2point): " << metricA.c2c_hausdorff_psnr << endl;
    if (!cPar.c2c_only)
    {
      cout << "   h.       1(p2plane): " << metricA.c2p_hausdorff << endl;
      cout << "   h.,PSNR  1(p2plane): " << metricA.c2p_hausdorff_psnr << endl;
    }
  }
  if ( cPar.bColor )
  {
    cout << "   c[0],    1         : " << metricA.color_mse[0] << endl;
    cout << "   c[1],    1         : " << metricA.color_mse[1] << endl;
    cout << "   c[2],    1         : " << metricA.color_mse[2] << endl;
    cout << "   c[0],PSNR1         : " << metricA.color_psnr[0] << endl;
    cout << "   c[1],PSNR1         : " << metricA.color_psnr[1] << endl;
    cout << "   c[2],PSNR1         : " << metricA.color_psnr[2] << endl;
    if ( cPar.hausdorff )
    {
      cout << " h.c[0],    1         : " << metricA.color_rgb_hausdorff[0] << endl;
      cout << " h.c[1],    1         : " << metricA.color_rgb_hausdorff[1] << endl;
      cout << " h.c[2],    1         : " << metricA.color_rgb_hausdorff[2] << endl;
      cout << " h.c[0],PSNR1         : " << metricA.color_rgb_hausdorff_psnr[0] << endl;
      cout << " h.c[1],PSNR1         : " << metricA.color_rgb_hausdorff_psnr[1] << endl;
      cout << " h.c[2],PSNR1         : " << metricA.color_rgb_hausdorff_psnr[2] << endl;
    }
  }
  if ( cPar.bLidar )
  {
    cout << "   r,       1         : " << metricA.reflectance_mse  << endl;
    cout << "   r,PSNR   1         : " << metricA.reflectance_psnr << endl;
    if ( cPar.hausdorff )
    {
      cout << " h.r,       1         : " << metricA.reflectance_hausdorff  << endl;
      cout << " h.r,PSNR   1         : " << metricA.reflectance_hausdorff_psnr << endl;
    }
  }

  if (!cPar.singlePass)
  {
    // Use "b" as reference
    cout << "2. Use infile2 (B) as reference, loop over B, use normals on A. (B->A).\n";
    qMetric metricB;
    metricB.pPSNR = pPSNR;
    findMetric( cloudB, cloudA, cPar, cloudNormalsA, metricB );

    cout << "   mse2      (p2point): " << metricB.c2c_mse << endl;
    cout << "   mse2,PSNR (p2point): " << metricB.c2c_psnr << endl;
	cout << "   mmd       (p2distr): " << metricB.mmd << endl;
	cout << "   mmd,PSNR  (p2distr): " << metricB.mmd_psnr << endl;
	cout << "   msmd      (p2distr): " << metricB.msmd << endl;
	cout << "   msmd,PSNR (p2distr): " << metricB.msmd_psnr << endl;
    if (!cPar.c2c_only)
    {
      cout << "   mse2      (p2plane): " << metricB.c2p_mse << endl;
      cout << "   mse2,PSNR (p2plane): " << metricB.c2p_psnr << endl;
    }
    if ( cPar.hausdorff )
    {
      cout << "   h.       2(p2point): " << metricB.c2c_hausdorff << endl;
      cout << "   h.,PSNR  2(p2point): " << metricB.c2c_hausdorff_psnr << endl;
      if (!cPar.c2c_only)
      {
        cout << "   h.       2(p2plane): " << metricB.c2p_hausdorff << endl;
        cout << "   h.,PSNR  2(p2plane): " << metricB.c2p_hausdorff_psnr << endl;
      }
    }
    if ( cPar.bColor)
    {
      cout << "   c[0],    2         : " << metricB.color_mse[0] << endl;
      cout << "   c[1],    2         : " << metricB.color_mse[1] << endl;
      cout << "   c[2],    2         : " << metricB.color_mse[2] << endl;
      cout << "   c[0],PSNR2         : " << metricB.color_psnr[0] << endl;
      cout << "   c[1],PSNR2         : " << metricB.color_psnr[1] << endl;
      cout << "   c[2],PSNR2         : " << metricB.color_psnr[2] << endl;
      if ( cPar.hausdorff)
      {
        cout << " h.c[0],    2         : " << metricB.color_rgb_hausdorff[0] << endl;
        cout << " h.c[1],    2         : " << metricB.color_rgb_hausdorff[1] << endl;
        cout << " h.c[2],    2         : " << metricB.color_rgb_hausdorff[2] << endl;
        cout << " h.c[0],PSNR2         : " << metricB.color_rgb_hausdorff_psnr[0] << endl;
        cout << " h.c[1],PSNR2         : " << metricB.color_rgb_hausdorff_psnr[1] << endl;
        cout << " h.c[2],PSNR2         : " << metricB.color_rgb_hausdorff_psnr[2] << endl;
      }
    }
    if ( cPar.bLidar )
    {
      cout << "   r,       2         : " << metricB.reflectance_mse  << endl;
      cout << "   r,PSNR   2         : " << metricB.reflectance_psnr << endl;
      if ( cPar.hausdorff )
      {
        cout << " h.r,       2         : " << metricB.reflectance_hausdorff  << endl;
        cout << " h.r,PSNR   2         : " << metricB.reflectance_hausdorff_psnr << endl;
      }

    }

    // Derive the final symmetric metric
    qual_metric.c2c_mse = max( metricA.c2c_mse, metricB.c2c_mse );
    qual_metric.c2p_mse = max( metricA.c2p_mse, metricB.c2p_mse );
    qual_metric.c2c_psnr = min( metricA.c2c_psnr, metricB.c2c_psnr );
    qual_metric.c2p_psnr = min( metricA.c2p_psnr, metricB.c2p_psnr );

    qual_metric.c2c_hausdorff = max( metricA.c2c_hausdorff, metricB.c2c_hausdorff  );
    qual_metric.c2p_hausdorff = max( metricA.c2p_hausdorff, metricB.c2p_hausdorff );
    qual_metric.c2c_hausdorff_psnr = min( metricA.c2c_hausdorff_psnr, metricB.c2c_hausdorff_psnr );
    qual_metric.c2p_hausdorff_psnr = min( metricA.c2p_hausdorff_psnr, metricB.c2p_hausdorff_psnr );

    //point-to-distribution
    qual_metric.mmd = max(metricA.mmd, metricB.mmd);
    qual_metric.msmd = max(metricA.msmd, metricB.msmd);
    qual_metric.mmd_psnr = min(metricA.mmd_psnr, metricB.mmd_psnr);
    qual_metric.msmd_psnr = min(metricA.msmd_psnr, metricB.msmd_psnr);

    if ( cPar.bColor )
    {
      qual_metric.color_mse[0] = max( metricA.color_mse[0], metricB.color_mse[0] );
      qual_metric.color_mse[1] = max( metricA.color_mse[1], metricB.color_mse[1] );
      qual_metric.color_mse[2] = max( metricA.color_mse[2], metricB.color_mse[2] );

      qual_metric.color_psnr[0] = min( metricA.color_psnr[0], metricB.color_psnr[0] );
      qual_metric.color_psnr[1] = min( metricA.color_psnr[1], metricB.color_psnr[1] );
      qual_metric.color_psnr[2] = min( metricA.color_psnr[2], metricB.color_psnr[2] );

      qual_metric.color_rgb_hausdorff[0] = max( metricA.color_rgb_hausdorff[0], metricB.color_rgb_hausdorff[0] );
      qual_metric.color_rgb_hausdorff[1] = max( metricA.color_rgb_hausdorff[1], metricB.color_rgb_hausdorff[1] );
      qual_metric.color_rgb_hausdorff[2] = max( metricA.color_rgb_hausdorff[2], metricB.color_rgb_hausdorff[2] );

      qual_metric.color_rgb_hausdorff_psnr[0] = min( metricA.color_rgb_hausdorff_psnr[0], metricB.color_rgb_hausdorff_psnr[0] );
      qual_metric.color_rgb_hausdorff_psnr[1] = min( metricA.color_rgb_hausdorff_psnr[1], metricB.color_rgb_hausdorff_psnr[1] );
      qual_metric.color_rgb_hausdorff_psnr[2] = min( metricA.color_rgb_hausdorff_psnr[2], metricB.color_rgb_hausdorff_psnr[2] );
    }
    if ( cPar.bLidar )
    {
      qual_metric.reflectance_mse  = max( metricA.reflectance_mse,  metricB.reflectance_mse  );
      qual_metric.reflectance_psnr = min( metricA.reflectance_psnr, metricB.reflectance_psnr );
      qual_metric.reflectance_hausdorff  = max( metricA.reflectance_hausdorff,  metricB.reflectance_hausdorff  );
      qual_metric.reflectance_hausdorff_psnr = min( metricA.reflectance_hausdorff_psnr, metricB.reflectance_hausdorff_psnr );
    }

    cout << "3. Final (symmetric).\n";
    cout << "   mseF      (p2point): " << qual_metric.c2c_mse << endl;
    cout << "   mseF,PSNR (p2point): " << qual_metric.c2c_psnr << endl;
	cout << "   mmd       (p2distr): " << qual_metric.mmd << endl;
	cout << "   mmd,PSNR  (p2distr): " << qual_metric.mmd_psnr << endl;
	cout << "   msmd      (p2distr): " << qual_metric.msmd << endl;
	cout << "   msmd,PSNR (p2distr): " << qual_metric.msmd_psnr << endl;
	
    if (!cPar.c2c_only)
    {
      cout << "   mseF      (p2plane): " << qual_metric.c2p_mse << endl;
      cout << "   mseF,PSNR (p2plane): " << qual_metric.c2p_psnr << endl;
    }
    if ( cPar.hausdorff )
    {
      cout << "   h.        (p2point): " << qual_metric.c2c_hausdorff << endl;
      cout << "   h.,PSNR   (p2point): " << qual_metric.c2c_hausdorff_psnr << endl;
      if (!cPar.c2c_only)
      {
        cout << "   h.        (p2plane): " << qual_metric.c2p_hausdorff << endl;
        cout << "   h.,PSNR   (p2plane): " << qual_metric.c2p_hausdorff_psnr << endl;
      }
    }
    if ( cPar.bColor )
    {
      cout << "   c[0],    F         : " << qual_metric.color_mse[0] << endl;
      cout << "   c[1],    F         : " << qual_metric.color_mse[1] << endl;
      cout << "   c[2],    F         : " << qual_metric.color_mse[2] << endl;
      cout << "   c[0],PSNRF         : " << qual_metric.color_psnr[0] << endl;
      cout << "   c[1],PSNRF         : " << qual_metric.color_psnr[1] << endl;
      cout << "   c[2],PSNRF         : " << qual_metric.color_psnr[2] << endl;
      if ( cPar.hausdorff )
      {
        cout << " h.c[0],    F         : " << qual_metric.color_rgb_hausdorff[0] << endl;
        cout << " h.c[1],    F         : " << qual_metric.color_rgb_hausdorff[1] << endl;
        cout << " h.c[2],    F         : " << qual_metric.color_rgb_hausdorff[2] << endl;
        cout << " h.c[0],PSNRF         : " << qual_metric.color_rgb_hausdorff_psnr[0] << endl;
        cout << " h.c[1],PSNRF         : " << qual_metric.color_rgb_hausdorff_psnr[1] << endl;
        cout << " h.c[2],PSNRF         : " << qual_metric.color_rgb_hausdorff_psnr[2] << endl;
      }
    }
    if ( cPar.bLidar )
    {
      cout << "   r,       F         : " << qual_metric.reflectance_mse  << endl;
      cout << "   r,PSNR   F         : " << qual_metric.reflectance_psnr << endl;
      if ( cPar.hausdorff )
      {
        cout << " h.r,       F         : " << qual_metric.reflectance_hausdorff  << endl;
        cout << " h.r,PSNR   F         : " << qual_metric.reflectance_hausdorff_psnr << endl;
      }
    }
  }
}
