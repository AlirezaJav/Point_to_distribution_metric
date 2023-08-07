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

#ifndef PCC_PROCESSING_HPP
#define PCC_PROCESSING_HPP

#include <array>
#include <initializer_list>
#include <iostream>
#include <fstream>
#include <memory>
#include <mutex>
#include <vector>

#define DUPLICATECOLORS_DEBUG 0 // 0 or 1

using namespace std;

namespace pcc_processing {

  class PccPointCloud;

  class PointBaseSet
  {
  public:
    PointBaseSet() {};
    virtual int loadPoints( PccPointCloud *pPcc, long int idx ) = 0;
  };

  class PointXYZSet : public PointBaseSet
  {
  private:
    int idxInLine[3];           //! index of the x,y,z in the line
  public:
    typedef std::array<float, 3> point_type;
    vector<point_type> p;
    vector< int > nbdup;
    PointXYZSet() {};
    ~PointXYZSet();
    virtual int loadPoints( PccPointCloud *pPcc, long int idx );
    void init( long int size, int i0 = -1, int i1 = -1, int i2 = -1 );
  };

  class RGBSet : public PointBaseSet
  {
  private:
    int idxInLine[3];           //! index of the r,g,b in the line
  public:
    typedef std::array<unsigned char, 3> value_type;
    vector<value_type> c;
    RGBSet() {}
    ~RGBSet();
    virtual int loadPoints( PccPointCloud *pPcc, long int idx );
    void init( long int size, int i0 = -1, int i1 = -1, int i2 = -1 );
  };

  class NormalSet : public PointBaseSet
  {
  private:
    int idxInLine[3];           //! index of the x,y,z in the line
  public:
    typedef std::array<float, 3> value_type;
    vector<value_type> n;
    NormalSet() { }
    ~NormalSet();
    virtual int loadPoints( PccPointCloud *pPcc, long int idx );
    void init( long int size, int i0 = -1, int i1 = -1, int i2 = -1 );
  };

  class LidarSet : public PointBaseSet
  {
  private:
    int idxInLine[1];           //! index of the reflectance in the line
  public:
    typedef unsigned short value_type;
    vector<value_type> reflectance;
    LidarSet() { }
    ~LidarSet();
    virtual int loadPoints( PccPointCloud *pPcc, long int idx );
    void init( long int size, int i0 = -1 );
  };

  struct ProxyIterator;
  struct Proxy;
  struct ProxyIterator {
    typedef std::random_access_iterator_tag iterator_category;
    typedef Proxy value_type;
    typedef std::ptrdiff_t difference_type;
    typedef Proxy* pointer;
    typedef Proxy& reference;

    ProxyIterator() : cloud(nullptr), idx() {}
    ProxyIterator(PccPointCloud* cloud) : cloud(cloud), idx() {}
    ProxyIterator(PccPointCloud* cloud, size_t idx) : cloud(cloud), idx(idx) {}

    ProxyIterator(const ProxyIterator&) = default;
    ProxyIterator& operator=(const ProxyIterator&) = default;

    ProxyIterator(ProxyIterator&&) = default;
    ProxyIterator& operator=(ProxyIterator&&) = default;

    bool operator==(const ProxyIterator& other) const;

    /* Iterator */
    value_type operator*() const;
    ProxyIterator& operator++();

    /* InputIterator */
    bool operator!=(const ProxyIterator& other) const;
    value_type operator->() const;
    ProxyIterator operator++(int);

    /* BidirectionalIterator */
    ProxyIterator& operator--();
    ProxyIterator operator--(int);

    /* RandomAccessIterator */
    ProxyIterator& operator+=(difference_type m);
    ProxyIterator operator+(difference_type m) const;
    ProxyIterator& operator-=(difference_type m);
    ProxyIterator operator-(difference_type m) const;
    difference_type operator-(const ProxyIterator& other) const;
    value_type operator[](size_t idx) const;
    bool operator<(const ProxyIterator& other) const;
    bool operator>(const ProxyIterator& other) const;
    bool operator<=(const ProxyIterator& other) const;
    bool operator>=(const ProxyIterator& other) const;

    PccPointCloud* cloud;
    size_t idx;
  };

  struct Proxy {
    Proxy() : it() {}
    Proxy(const ProxyIterator& it) : it(it) {}

    Proxy(const Proxy&) = delete;
    Proxy& operator=(const Proxy&) = delete;

    Proxy& operator=(Proxy&& other);
    Proxy(Proxy&& other)
    : it() {
      *this = std::move(other);
    }

    void swap(Proxy& other);

    bool operator<(const Proxy& other) const;
    bool operator==(const Proxy& other) const;

    const ProxyIterator it;

    PointXYZSet::point_type xyz;
    int nbdup;
    NormalSet::value_type normal;
    RGBSet::value_type rgb;
    LidarSet::value_type lidar;
  };

  /**!
   * \brief
   *  Wrapper class to hold different type of attributes
   *  Easier to include new types of attributes
   *  
   *  Dong Tian <tian@merl.com>
   */
  class PccPointCloud
  {
  public:
    enum PointFieldTypes { INT8 = 1,
                           UINT8 = 2,
                           INT16 = 3,
                           UINT16 = 4,
                           INT32 = 5,
                           UINT32 = 6,
                           FLOAT32 = 7,
                           FLOAT64 = 8 };

    long int size;                 //! The number of points
    int fileFormat;                //! 0: ascii. 1: binary_little_endian
    std::vector<int> fieldType;    //! The field type
    std::vector<int> fieldPos;     //! The field position in the line memory
    int fieldNum;                  //! The number of field available
    int fieldSize;                 //! The memory size of the used fields
    int dataPos;                   //! The file pointer to the beginning of data
    int lineNum;                   //! The number of lines of header section
    std::unique_ptr<unsigned char[]> lineMem;

    int checkFile( string fileName );
    int checkField( string fileName, string fieldName, const std::initializer_list<const char*>& fieldTypes );
    int loadLine( ifstream &in );
#if _WIN32
    int seekBinary(ifstream &in);
    int seekAscii(ifstream &in);
#endif

    ProxyIterator begin() {
      return ProxyIterator(this, 0);
    }

    ProxyIterator end() {
      return ProxyIterator(this, size);
    }

  public:
    PointXYZSet xyz;
    RGBSet rgb;
    NormalSet normal;
    LidarSet lidar;

    bool bXyz;
    bool bRgb;
    bool bNormal;
    bool bLidar;

    PccPointCloud();
    ~PccPointCloud();
    int load( string inFile, bool normalsOnly = false, int dropDuplicates = 0, int neighborsProc = 0) ;
  };


// ------------- detail ----------------

  inline void swap(Proxy& a, Proxy& b) {
    a.swap(b);
  }

  inline void swap(Proxy&& a, Proxy&& b) {
    a.swap(b);
  }

  inline void Proxy::swap(Proxy& other) {
    /* swap proxied storage */
    auto& a = this->it.cloud;
    auto& b = other.it.cloud;

    auto& a_xyz = a ? a->xyz.p[this->it.idx] : this->xyz;
    auto& b_xyz = b ? b->xyz.p[other.it.idx] : other.xyz;

    auto& a_nbdup = a ? a->xyz.nbdup[this->it.idx] : this->nbdup;
    auto& b_nbdup = b ? b->xyz.nbdup[other.it.idx] : other.nbdup;

    auto& a_normal = a && a->bNormal ? a->normal.n[this->it.idx] : this->normal;
    auto& b_normal = b && b->bNormal ? b->normal.n[other.it.idx] : other.normal;

    auto& a_rgb = a && a->bRgb ? a->rgb.c[this->it.idx] : this->rgb;
    auto& b_rgb = b && b->bRgb ? b->rgb.c[other.it.idx] : other.rgb;

    auto& a_lidar = a && a->bLidar
      ? a->lidar.reflectance[this->it.idx] : this->lidar;
    auto& b_lidar = b && b->bLidar
      ? b->lidar.reflectance[other.it.idx] : other.lidar;

    using std::swap;
    swap(a_xyz, b_xyz);
    swap(a_nbdup, b_nbdup);
    swap(a_normal, b_normal);
    swap(a_rgb, b_rgb);
    swap(a_lidar, b_lidar);
  }

  inline Proxy& Proxy::operator=(Proxy&& other) {
    /* move operation */
    auto& a = this->it.cloud;
    auto& b = other.it.cloud;

    auto& a_xyz = a ? a->xyz.p[this->it.idx] : this->xyz;
    auto& b_xyz = b ? b->xyz.p[other.it.idx] : other.xyz;

    auto& a_nbdup = a ? a->xyz.nbdup[this->it.idx] : this->nbdup;
    auto& b_nbdup = b ? b->xyz.nbdup[other.it.idx] : other.nbdup;

    auto& a_normal = a && a->bNormal ? a->normal.n[this->it.idx] : this->normal;
    auto& b_normal = b && b->bNormal ? b->normal.n[other.it.idx] : other.normal;

    auto& a_rgb = a && a->bRgb ? a->rgb.c[this->it.idx] : this->rgb;
    auto& b_rgb = b && b->bRgb ? b->rgb.c[other.it.idx] : other.rgb;

    auto& a_lidar = a && a->bLidar
      ? a->lidar.reflectance[this->it.idx] : this->lidar;
    auto& b_lidar = b && b->bLidar
      ? b->lidar.reflectance[other.it.idx] : other.lidar;

    a_xyz = std::move(b_xyz);
    a_nbdup = std::move(b_nbdup);
    a_normal = std::move(b_normal);
    a_rgb = std::move(b_rgb);
    a_lidar = std::move(b_lidar);

    return *this;
  }

  inline bool Proxy::operator<(const Proxy& other) const {
    auto& a = this->it.cloud;
    auto& b = other.it.cloud;

    auto& a_xyz = a ? a->xyz.p[this->it.idx] : this->xyz;
    auto& b_xyz = b ? b->xyz.p[other.it.idx] : other.xyz;

    return a_xyz < b_xyz;
  }

  inline bool Proxy::operator==(const Proxy& other) const {
    auto& a = this->it.cloud;
    auto& b = other.it.cloud;

    auto& a_xyz = a ? a->xyz.p[this->it.idx] : this->xyz;
    auto& b_xyz = b ? b->xyz.p[other.it.idx] : other.xyz;

    return a_xyz == b_xyz;
  }

// ----

  inline bool ProxyIterator::operator==(const ProxyIterator& other) const {
    return idx == other.idx && cloud == other.cloud;
  }

  inline bool ProxyIterator::operator!=(const ProxyIterator& other) const {
    return !(*this == other);
  }

  inline ProxyIterator::value_type ProxyIterator::operator*() const {
    return Proxy(*this);
  }

  inline ProxyIterator& ProxyIterator::operator++() {
    idx++;
    return *this;
  }

  inline ProxyIterator ProxyIterator::operator++(int) {
    ProxyIterator it(*this);
    idx++;
    return it;
  }

  inline ProxyIterator& ProxyIterator::operator--() {
    idx--;
    return *this;
  }

  inline ProxyIterator ProxyIterator::operator--(int) {
    ProxyIterator it(*this);
    idx--;
    return it;
  }

  inline ProxyIterator& ProxyIterator::operator+=(ProxyIterator::difference_type m) {
    idx += m;
    return *this;
  }

  inline ProxyIterator ProxyIterator::operator+(ProxyIterator::difference_type m) const {
    ProxyIterator it(*this);
    it += m;
    return it;
  }

  inline ProxyIterator& ProxyIterator::operator-=(ProxyIterator::difference_type m) {
    return *this += -m;
  }

  inline ProxyIterator ProxyIterator::operator-(ProxyIterator::difference_type m) const {
    return *this + (-m);
  }

  inline ProxyIterator::difference_type ProxyIterator::operator-(const ProxyIterator& other) const {
    return ptrdiff_t(idx - other.idx);
  }

  inline ProxyIterator::value_type ProxyIterator::operator[](size_t idx) const {
    return Proxy(*this);
  }

  inline bool ProxyIterator::operator<(const ProxyIterator& other) const {
    return idx < other.idx;
  }

  inline bool ProxyIterator::operator>(const ProxyIterator& other) const {
    return idx > other.idx;
  }

  inline bool ProxyIterator::operator<=(const ProxyIterator& other) const {
    return !(*this > other);
  }

  inline bool ProxyIterator::operator>=(const ProxyIterator& other) const {
    return !(*this < other);
  }
};

#endif
