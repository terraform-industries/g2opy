#ifndef G2O_ARUCO_MARKER
#define G2O_ARUCO_MARKER
#include <unordered_map>
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

template <typename Derived>
void alignZ(const Eigen::MatrixBase<Derived> &R)
{
    const auto z = R.col(2);
    const auto a = acos(z.z());
    auto c = Eigen::Vector3d(0.0, 0.0, 1.0).cross(z).normalized();

    Eigen::Matrix3d r = Eigen::AngleAxisd(a, c).toRotationMatrix();
    const_cast<Eigen::MatrixBase<Derived> &>(R) = r.transpose() * R;
}

class G2O_TYPES_SLAM3D_ADDONS_API VertexArucoObject : public VertexSE3
{
public:
    VertexArucoObject() : VertexSE3(), obj_points_per_marker({}) {}
    VertexArucoObject(const std::unordered_map<std::uint32_t, CornerXYZ>& m, bool only4dof_) : VertexSE3(), obj_points_per_marker(m), only4dof(only4dof_) {}

    CornerXYZ worldCornersMarker(std::uint32_t marker_id)
    {
        const CornerXYZ& obj_points = obj_points_per_marker[marker_id];
        CornerXYZ world_corners;
        world_corners.transpose() = _estimate * obj_points.transpose();
        return world_corners;
    }

    virtual void oplusImpl(const double* update) {
        VertexSE3::oplusImpl(update);
        alignZ(_estimate.matrix().topLeftCorner<3, 3>());
    }

private:
    std::unordered_map<std::uint32_t, CornerXYZ> obj_points_per_marker;
    bool only4dof;
};

class G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3ArucoObject : public BaseBinaryEdge<12, CornerXYZ, VertexSE3, VertexArucoObject> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3ArucoObject(): BaseBinaryEdge<12, CornerXYZ, VertexSE3, VertexArucoObject>(), marker_id(9999) {
        resizeParameters(1);
        installParameter(params, 0);
        information().setIdentity();
    }

    EdgeSE3ArucoObject(std::uint32_t marker_id_): BaseBinaryEdge<12, CornerXYZ, VertexSE3, VertexArucoObject>(), marker_id(marker_id_) {
        resizeParameters(1);
        installParameter(params, 0);
        information().setIdentity();
    }

    virtual bool read(std::istream& /*is*/) { return true; }
    virtual bool write(std::ostream& /*os*/) const { return true; }



    void computeError() {
        VertexArucoObject* object = static_cast<VertexArucoObject*>(_vertices[1]);
        auto corners = object->worldCornersMarker(marker_id);

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
    std::uint32_t marker_id;
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