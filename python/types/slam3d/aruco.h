#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <g2o/types/slam3d_addons/aruco.h>


namespace py = pybind11;
using namespace pybind11::literals;


namespace g2o {

void declareAruco(py::module & m) {

    py::class_<VertexArucoMarker, BaseVertex<6, Isometry3D>>(m, "VertexArucoMarker")
        .def(py::init<double>(), "size"_a)
        .def("cornerXYZ", &VertexArucoMarker::cornerXYZ)
        .def("oplus_impl", &VertexSE3::oplusImpl)
    ;

    py::class_<VertexArucoObject, BaseVertex<6, Isometry3D>>(m, "VertexArucoObject")
        .def(py::init<std::unordered_map<std::uint32_t, CornerXYZ>>(), "obj_points"_a)
        .def("cornerXYZ", &VertexArucoObject::worldCornersMarker)
    ;

    templatedBaseEdge<12, CornerXYZ>(m, "_3_CornerXYZ");
    templatedBaseBinaryEdge<12, CornerXYZ, VertexSE3, VertexArucoMarker>(m, "_12_CornerXYZ_VertexSE3_VertexArucoMarker");
    py::class_<EdgeSE3ArucoMarker, BaseBinaryEdge<12, CornerXYZ, VertexSE3, VertexArucoMarker>>(m, "EdgeSE3ArucoMarker")
        .def(py::init<>())
        .def("compute_error", &EdgeSE3ArucoMarker::computeError)
        .def("set_measurement", &EdgeSE3ArucoMarker::setMeasurement)
    ;

    templatedBaseBinaryEdge<12, CornerXYZ, VertexSE3, VertexArucoObject>(m, "_12_CornerXYZ_VertexSE3_VertexArucoObject");
    py::class_<EdgeSE3ArucoObject, BaseBinaryEdge<12, CornerXYZ, VertexSE3, VertexArucoObject>>(m, "EdgeSE3ArucoObject")
        .def(py::init<std::uint32_t>(), "marker_id"_a)
        .def("compute_error", &EdgeSE3ArucoObject::computeError)
        .def("set_measurement", &EdgeSE3ArucoObject::setMeasurement)
    ;
}

}  // end namespace g2o