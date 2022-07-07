#ifndef G2O_ARUCO_MARKER
#define G2O_ARUCO_MARKER

#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam3d_addons_api.h"
#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/parameter_camera.h"

namespace g2o
{
using CornerXYZ = Eigen::Matrix<double, 4, 3>;

class G2O_TYPES_SLAM3D_ADDONS_API VertexArucoMarker : public VertexSE3
{
public:
    VertexArucoMarker() : VertexSE3(), size(0.0) {}
    VertexArucoMarker(double _size) : VertexSE3(), size(_size) {}

    CornerXYZ cornerXYZ()
    {
        double h = size * 0.5;
        CornerXYZ corners;
        corners << -h, h, 0.0,
            h, h, 0.0,
            h, -h, 0.0,
            -h, -h, 0.0;
        corners.transpose() = _estimate * corners.transpose();
        return corners;
    }

private:
    double size;
};

class G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3ArucoMarker : public BaseBinaryEdge<12, CornerXYZ, VertexSE3, VertexArucoMarker> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3ArucoMarker(): BaseBinaryEdge<12, CornerXYZ, VertexSE3, VertexArucoMarker>() {
        resizeParameters(1);
        installParameter(params, 0);
        information().setIdentity();
    }

    virtual bool read(std::istream& /*is*/) { return true; }
    virtual bool write(std::ostream& /*os*/) const { return true; }



    void computeError() {
        VertexArucoMarker* marker = static_cast<VertexArucoMarker*>(_vertices[1]);
        auto corners = marker->cornerXYZ();

        Eigen::Matrix<double, 3, 4> transposedMeas = _measurement.transpose();
        Eigen::Map<Eigen::Matrix<double, 12, 1>> m(transposedMeas.data(), transposedMeas.size());

        Eigen::Matrix<double, 12, 1> expected;
        for (size_t i = 0; i < 4; i++) {
            Vector3D p = cache->w2i() * corners.row(i).transpose();
            expected(i * 3 + 0) = p(0)/p(2);
            expected(i * 3 + 1) = p(1)/p(2);
            // Set depth to 0 if measurement also has no depth
            expected(i * 3 + 2) = m(i * 3 + 2) != 0 ? p(2) : 0;
        }


        // error, which is backwards from the normal observed - calculated
        // NOTE(joris): Why?
        _error = expected.array() - m.array();
    }

    virtual void setMeasurement(const CornerXYZ& m) {
      _measurement = m;
    }

    virtual bool getMeasurementData(double* d) const{
      Eigen::Map<CornerXYZ> v(d);
      v=_measurement;
      return true;
    }

    virtual int measurementDimension() const { return 12; }
private:
    ParameterCamera* params;
    CacheCamera* cache;
    virtual bool resolveCaches() {
        ParameterVector pv(1);
        pv[0] = params;
        resolveCache(cache, (OptimizableGraph::Vertex*)_vertices[0],"CACHE_CAMERA",pv);
        return cache != 0;
    }
};
}

#endif