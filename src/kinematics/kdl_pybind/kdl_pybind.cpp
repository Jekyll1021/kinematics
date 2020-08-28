//Copyright  (C)  2020  Ruben Smits <ruben dot smits at intermodalics dot eu>
//
//Version: 1.0
//Author: Ruben Smits Ruben Smits <ruben dot smits at intermodalics dot eu>
//Author: Zihan Chen <zihan dot chen dot jhu at gmail dot com>
//Author: Matthijs van der Burgh <MatthijsBurgh at outlook dot com>
//Maintainer: Ruben Smits Ruben Smits <ruben dot smits at intermodalics dot eu>
//URL: http://www.orocos.org/kdl
//
//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public
//License as published by the Free Software Foundation; either
//version 2.1 of the License, or (at your option) any later version.
//
//This library is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//Lesser General Public License for more details.
//
//You should have received a copy of the GNU Lesser General Public
//License along with this library; if not, write to the Free Software
//Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


#include <kdl/config.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <iostream>
#include <iomanip>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/framevel.hpp>
#include <kdl/framevel_io.hpp>
#include <kdl/joint.hpp>
#include <kdl/rotationalinertia.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <pybind11/pybind11.h>

namespace py = pybind11;


PYBIND11_MODULE(kdl_pybind, m) {
	// --------------
	// frames.hpp
	// --------------

	// Vector
	py::class_<KDL::Vector> vector(m, "Vector");
    vector.def(py::init<>());
    vector.def(py::init<double, double, double>());
    vector.def(py::init<const KDL::Vector&>());
    vector.def("x", (void (KDL::Vector::*)(double)) &KDL::Vector::x);
    vector.def("y", (void (KDL::Vector::*)(double)) &KDL::Vector::y);
    vector.def("z", (void (KDL::Vector::*)(double)) &KDL::Vector::z);
    vector.def("x", (double (KDL::Vector::*)(void) const) &KDL::Vector::x);
    vector.def("y", (double (KDL::Vector::*)(void) const) &KDL::Vector::y);
    vector.def("z", (double (KDL::Vector::*)(void) const) &KDL::Vector::z);
    vector.def("__getitem__", [](const KDL::Vector &v, int i)
    {
        if (i < 0 || i > 2)
            throw py::index_error("Vector index out of range");

        return v(i);
    });
    vector.def("__setitem__", [](KDL::Vector &v, int i, double value)
    {
        if (i < 0 || i > 2)
            throw py::index_error("Vector index out of range");

        v(i) = value;
    });
    vector.def("__repr__", [](const KDL::Vector &v)
    {
        std::ostringstream oss;
        oss << v;
        return oss.str();
    });
    vector.def("ReverseSign", &KDL::Vector::ReverseSign);

    vector.def("__mul__", [](const KDL::Vector &a, const KDL::Vector &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    vector.def("__mul__", [](const KDL::Vector &a, double &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    vector.def("__mul__", [](double &a, const KDL::Vector &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    vector.def("__iadd__", (KDL::Vector (KDL::Vector::*)(const KDL::Vector&) const) &KDL::Vector::operator+=, py::is_operator());
    vector.def("__isub__", (KDL::Vector (KDL::Vector::*)(const KDL::Vector&) const) &KDL::Vector::operator-=, py::is_operator());

    vector.def("__add__", [](const KDL::Vector &a, const KDL::Vector &b)
    {
        return operator+(a, b);
    }, py::is_operator());

    vector.def("__sub__", [](const KDL::Vector &a, const KDL::Vector &b)
    {
        return operator-(a, b);
    }, py::is_operator());

    vector.def("__truediv__", [](const KDL::Vector &a, double &b)
    {
        return operator/(a, b);
    }, py::is_operator());

    vector.def("__eq__", [](const KDL::Vector &a, const KDL::Vector &b)
    {
        return operator==(a, b);
    }, py::is_operator());

    vector.def("__ne__", [](const KDL::Vector &a, const KDL::Vector &b)
    {
        return operator!=(a, b);
    }, py::is_operator());

    vector.def("__neg__", [](const KDL::Vector &a)
    {
        return operator-(a);
    }, py::is_operator());
    vector.def("__copy__", [](const KDL::Vector& self)
    {
        return KDL::Vector(self);
    });
    vector.def("__deepcopy__", [](const KDL::Vector& self, py::dict)
    {
        return KDL::Vector(self);
    }, py::arg("memo"));
    vector.def_static("Zero", &KDL::Vector::Zero);
    vector.def("Norm", &KDL::Vector::Norm);
    vector.def("Normalize", &KDL::Vector::Normalize, py::arg("eps")=KDL::epsilon);
    vector.def(py::pickle(
            [](const KDL::Vector &v)
            { // __getstate__
                /* Return a tuple that fully encodes the state of the object */
                return py::make_tuple(v.x(), v.y(), v.z());
            },
            [](py::tuple t)
            { // __setstate__
                if (t.size() != 3)
                    throw std::runtime_error("Invalid state!");

                /* Create a new C++ instance */
                KDL::Vector v(t[0].cast<double>(), t[1].cast<double>(), t[2].cast<double>());
                return v;
            }));

    m.def("SetToZero", (void (*)(KDL::Vector&)) &KDL::SetToZero);
    m.def("dot", (double (*)(const KDL::Vector&, const KDL::Vector&)) &KDL::dot);
    m.def("Equal", (bool (*)(const KDL::Vector&, const KDL::Vector&, double)) &KDL::Equal,
          py::arg("a"), py::arg("b"), py::arg("eps")=KDL::epsilon);

    // Wrench
    py::class_<KDL::Wrench> wrench(m, "Wrench");
    wrench.def(py::init<>());
    wrench.def(py::init<const KDL::Vector&, const KDL::Vector&>());
    wrench.def(py::init<const KDL::Wrench&>());
    wrench.def_readwrite("force", &KDL::Wrench::force);
    wrench.def_readwrite("torque", &KDL::Wrench::torque);
    wrench.def("__getitem__", [](const KDL::Wrench &t, int i)
    {
        if (i < 0 || i > 5)
            throw py::index_error("Wrench index out of range");

        return t(i);
    });
    wrench.def("__setitem__", [](KDL::Wrench &t, int i, double value)
    {
        if (i < 0 || i > 5)
            throw py::index_error("Wrench index out of range");

        t(i) = value;
    });
    wrench.def("__repr__", [](const KDL::Wrench &t)
    {
        std::ostringstream oss;
        oss << t;
        return oss.str();
    });
    wrench.def("__copy__", [](const KDL::Wrench& self)
    {
        return KDL::Wrench(self);
    });
    wrench.def("__deepcopy__", [](const KDL::Wrench& self, py::dict)
    {
        return KDL::Wrench(self);
    }, py::arg("memo"));
    wrench.def_static("Zero", &KDL::Wrench::Zero);
    wrench.def("ReverseSign", &KDL::Wrench::ReverseSign);
    wrench.def("RefPoint", &KDL::Wrench::RefPoint);

    wrench.def("__mul__", [](const KDL::Wrench &a, double &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    wrench.def("__mul__", [](double &a, const KDL::Wrench &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    wrench.def("__iadd__", (KDL::Wrench (KDL::Wrench::*)(const KDL::Wrench&) const) &KDL::Wrench::operator+=, py::is_operator());
    wrench.def("__isub__", (KDL::Wrench (KDL::Wrench::*)(const KDL::Wrench&) const) &KDL::Wrench::operator-=, py::is_operator());

    wrench.def("__add__", [](const KDL::Wrench &a, const KDL::Wrench &b)
    {
        return operator+(a, b);
    }, py::is_operator());

    wrench.def("__sub__", [](const KDL::Wrench &a, const KDL::Wrench &b)
    {
        return operator-(a, b);
    }, py::is_operator());

    wrench.def("__truediv__", [](const KDL::Wrench &a, double &b)
    {
        return operator/(a, b);
    }, py::is_operator());

    wrench.def("__eq__", [](const KDL::Wrench &a, const KDL::Wrench &b)
    {
        return operator==(a, b);
    }, py::is_operator());

    wrench.def("__ne__", [](const KDL::Wrench &a, const KDL::Wrench &b)
    {
        return operator!=(a, b);
    }, py::is_operator());

    wrench.def("__neg__", [](const KDL::Wrench &w)
    {
        return operator-(w);
    }, py::is_operator());
    wrench.def(py::pickle(
            [](const KDL::Wrench &wr)
            { // __getstate__
                /* Return a tuple that fully encodes the state of the object */
                return py::make_tuple(wr.force, wr.torque);
            },
            [](py::tuple t)
            { // __setstate__
                if (t.size() != 2)
                    throw std::runtime_error("Invalid state!");

                /* Create a new C++ instance */
                KDL::Wrench wr(t[0].cast<KDL::Vector>(), t[1].cast<KDL::Vector>());
                return wr;
            }));

    m.def("SetToZero", (void (*)(KDL::Wrench&)) &KDL::SetToZero);
    m.def("Equal", (bool (*)(const KDL::Wrench&, const KDL::Wrench&, double eps)) &KDL::Equal,
          py::arg("a"), py::arg("b"), py::arg("eps")=KDL::epsilon);

    // Twist

    py::class_<KDL::Twist> twist(m, "Twist");
    twist.def(py::init<>());
    twist.def(py::init<const KDL::Vector&, const KDL::Vector&>());
    twist.def(py::init<const KDL::Twist&>());
    twist.def_readwrite("vel", &KDL::Twist::vel);
    twist.def_readwrite("rot", &KDL::Twist::rot);
    twist.def("__getitem__", [](const KDL::Twist &t, int i)
    {
        if (i < 0 || i > 5)
            throw py::index_error("Twist index out of range");

        return t(i);
    });
    twist.def("__setitem__", [](KDL::Twist &t, int i, double value)
    {
        if (i < 0 || i > 5)
            throw py::index_error("Twist index out of range");

        t(i) = value;
    });
    twist.def("__repr__", [](const KDL::Twist &t)
    {
        std::ostringstream oss;
        oss << t;
        return oss.str();
    });
    twist.def("__copy__", [](const KDL::Twist& self)
    {
        return KDL::Twist(self);
    });
    twist.def("__deepcopy__", [](const KDL::Twist& self, py::dict)
    {
        return KDL::Twist(self);
    }, py::arg("memo"));
    twist.def_static("Zero", &KDL::Twist::Zero);
    twist.def("ReverseSign", &KDL::Twist::ReverseSign);
    twist.def("RefPoint", &KDL::Twist::RefPoint);

    twist.def("__mul__", [](const KDL::Twist &a, double &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    twist.def("__mul__", [](double &a, const KDL::Twist &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    twist.def("__iadd__", (KDL::Twist (KDL::Twist::*)(const KDL::Twist&) const) &KDL::Twist::operator+=, py::is_operator());
    twist.def("__isub__", (KDL::Twist (KDL::Twist::*)(const KDL::Twist&) const) &KDL::Twist::operator-=, py::is_operator());

    twist.def("__add__", [](const KDL::Twist &a, const KDL::Twist &b)
    {
        return operator+(a, b);
    }, py::is_operator());

    twist.def("__sub__", [](const KDL::Twist &a, const KDL::Twist &b)
    {
        return operator-(a, b);
    }, py::is_operator());

    twist.def("__truediv__", [](const KDL::Twist &a, double &b)
    {
        return operator/(a, b);
    }, py::is_operator());

    twist.def("__eq__", [](const KDL::Twist &a, const KDL::Twist &b)
    {
        return operator==(a, b);
    }, py::is_operator());

    twist.def("__ne__", [](const KDL::Twist &a, const KDL::Twist &b)
    {
        return operator!=(a, b);
    }, py::is_operator());

    twist.def("__neg__", [](const KDL::Twist &a)
    {
        return operator-(a);
    }, py::is_operator());
    twist.def(py::pickle(
            [](const KDL::Twist &tt)
            { // __getstate__
                /* Return a tuple that fully encodes the state of the object */
                return py::make_tuple(tt.vel, tt.rot);
            },
            [](py::tuple t)
            { // __setstate__
                if (t.size() != 2)
                    throw std::runtime_error("Invalid state!");

                /* Create a new C++ instance */
                KDL::Twist tt(t[0].cast<KDL::Vector>(), t[1].cast<KDL::Vector>());
                return tt;
            }));

    m.def("dot", (double (*)(const KDL::Twist&, const KDL::Wrench&)) &KDL::dot);
    m.def("dot", (double (*)(const KDL::Wrench&, const KDL::Twist&)) &KDL::dot);
    m.def("SetToZero", (void (*)(KDL::Twist&)) &KDL::SetToZero);
    m.def("Equal", (bool (*)(const KDL::Twist&, const KDL::Twist&, double eps)) &KDL::Equal,
          py::arg("a"), py::arg("b"), py::arg("eps")=KDL::epsilon);

	// Rotation
	py::class_<KDL::Rotation> rotation(m, "Rotation");
    rotation.def(py::init<>());
    rotation.def(py::init<double, double, double, double, double, double, double, double, double>());
    rotation.def(py::init<const KDL::Vector&, const KDL::Vector&, const KDL::Vector&>());
    rotation.def(py::init<const KDL::Rotation&>());
    rotation.def("__getitem__", [](const KDL::Rotation &r, std::tuple<int, int> idx)
    {
        int i = std::get<0>(idx);
        int j = std::get<1>(idx);
        if (i < 0 || i > 2 || j < 0 || j > 2)
            throw py::index_error("Rotation index out of range");

        return r(i, j);
    });
    rotation.def("__setitem__", [](KDL::Rotation &r, std::tuple<int, int> idx, double value)
    {
        int i = std::get<0>(idx);
        int j = std::get<1>(idx);
        if (i < 0 || i > 2 || j < 0 || j > 2)
            throw py::index_error("Rotation index out of range");

        r(i, j) = value;
    });
    rotation.def("__repr__", [](const KDL::Rotation &r)
    {
            std::ostringstream oss;
            oss << r;
            return oss.str();
    });
    rotation.def("__copy__", [](const KDL::Rotation& self)
    {
        return KDL::Rotation(self);
    });
    rotation.def("__deepcopy__", [](const KDL::Rotation& self, py::dict)
    {
        return KDL::Rotation(self);
    }, py::arg("memo"));

    rotation.def("SetInverse", &KDL::Rotation::SetInverse);
    rotation.def("Inverse", (KDL::Rotation (KDL::Rotation::*)(void) const) &KDL::Rotation::Inverse);
    rotation.def("Inverse", (KDL::Vector (KDL::Rotation::*)(const KDL::Vector&) const) &KDL::Rotation::Inverse);
    rotation.def("Inverse", (KDL::Wrench (KDL::Rotation::*)(const KDL::Wrench&) const) &KDL::Rotation::Inverse);
    rotation.def("Inverse", (KDL::Twist (KDL::Rotation::*)(const KDL::Twist&) const) &KDL::Rotation::Inverse);
    rotation.def_static("Identity", &KDL::Rotation::Identity);
    rotation.def_static("RotX", &KDL::Rotation::RotX);
    rotation.def_static("RotY", &KDL::Rotation::RotY);
    rotation.def_static("RotZ", &KDL::Rotation::RotZ);
    rotation.def_static("Rot", &KDL::Rotation::Rot);
    rotation.def_static("Rot2", &KDL::Rotation::Rot2);
    rotation.def_static("EulerZYZ", &KDL::Rotation::EulerZYZ);
    rotation.def_static("RPY", &KDL::Rotation::RPY);
    rotation.def_static("EulerZYX", &KDL::Rotation::EulerZYX);
    rotation.def_static("Quaternion", &KDL::Rotation::Quaternion);
    rotation.def("DoRotX", &KDL::Rotation::DoRotX);
    rotation.def("DoRotY", &KDL::Rotation::DoRotY);
    rotation.def("DoRotZ", &KDL::Rotation::DoRotZ);
    rotation.def("GetRot", &KDL::Rotation::GetRot);
    rotation.def("GetRotAngle", [](const KDL::Rotation &r, double eps)
    {
        KDL::Vector axis;
        double ret = r.GetRotAngle(axis, eps);
        return py::make_tuple(ret, axis);
    }, py::arg("eps") = KDL::epsilon);
    rotation.def("GetEulerZYZ", [](const KDL::Rotation &r)
    {
        double Alfa, Beta, Gamma;
        r.GetEulerZYZ(Alfa, Beta, Gamma);
        return py::make_tuple(Alfa, Beta, Gamma);
    });
    rotation.def("GetRPY", [](const KDL::Rotation &r)
    {
        double roll, pitch, yaw;
        r.GetRPY(roll, pitch, yaw);
        return py::make_tuple(roll, pitch, yaw);
    });
    rotation.def("GetEulerZYX", [](const KDL::Rotation &r)
    {
        double Alfa, Beta, Gamma;
        r.GetEulerZYX(Alfa, Beta, Gamma);
        return py::make_tuple(Alfa, Beta, Gamma);
    });
    rotation.def("GetQuaternion", [](const KDL::Rotation &r)
    {
        double x, y, z, w;
        r.GetQuaternion(x, y, z, w);
        return py::make_tuple(x, y, z, w);
    });
    rotation.def("UnitX", (KDL::Vector (KDL::Rotation::*)() const) &KDL::Rotation::UnitX);
    rotation.def("UnitY", (KDL::Vector (KDL::Rotation::*)() const) &KDL::Rotation::UnitY);
    rotation.def("UnitZ", (KDL::Vector (KDL::Rotation::*)() const) &KDL::Rotation::UnitZ);
    rotation.def("UnitX", (void (KDL::Rotation::*)(const KDL::Vector&)) &KDL::Rotation::UnitX);
    rotation.def("UnitY", (void (KDL::Rotation::*)(const KDL::Vector&)) &KDL::Rotation::UnitY);
    rotation.def("UnitZ", (void (KDL::Rotation::*)(const KDL::Vector&)) &KDL::Rotation::UnitZ);

    rotation.def("__mul__", (KDL::Vector (KDL::Rotation::*)(const KDL::Vector&) const) &KDL::Rotation::operator*, py::is_operator());
    rotation.def("__mul__", (KDL::Wrench (KDL::Rotation::*)(const KDL::Wrench&) const) &KDL::Rotation::operator*, py::is_operator());
    rotation.def("__mul__", (KDL::Twist (KDL::Rotation::*)(const KDL::Twist&) const) &KDL::Rotation::operator*, py::is_operator());

    rotation.def("__mul__", [](const KDL::Rotation &a, const KDL::Rotation &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    rotation.def("__mul__", [](const KDL::Rotation &a, const KDL::RigidBodyInertia &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    rotation.def("__eq__", [](const KDL::Rotation &a, const KDL::Rotation &b)
    {
        return operator==(a, b);
    }, py::is_operator());

    rotation.def("__ne__", [](const KDL::Rotation &a, const KDL::Rotation &b)
    {
        return operator!=(a, b);
    }, py::is_operator());

    rotation.def(py::pickle(
            [](const KDL::Rotation &rot)
            { // __getstate__
                /* Return a tuple that fully encodes the state of the object */
                double roll{0}, pitch{0}, yaw{0};
                rot.GetRPY(roll, pitch, yaw);
                return py::make_tuple(roll, pitch, yaw);
            },
            [](py::tuple t)
            { // __setstate__
                if (t.size() != 3)
                    throw std::runtime_error("Invalid state!");

                /* Create a new C++ instance */
                return KDL::Rotation::RPY(t[0].cast<double>(), t[1].cast<double>(), t[2].cast<double>());
            }));

    m.def("Equal", (bool (*)(const KDL::Rotation&, const KDL::Rotation&, double eps)) &KDL::Equal,
          py::arg("a"), py::arg("b"), py::arg("eps")=KDL::epsilon);

    // Frame
    py::class_<KDL::Frame> frame(m, "Frame");
    frame.def(py::init<const KDL::Rotation&, const KDL::Vector&>());
    frame.def(py::init<const KDL::Vector&>());
    frame.def(py::init<const KDL::Rotation&>());
    frame.def(py::init<const KDL::Frame&>());
    frame.def(py::init<>());
    frame.def_readwrite("M", &KDL::Frame::M);
    frame.def_readwrite("p", &KDL::Frame::p);
    frame.def("__getitem__", [](const KDL::Frame &frm, std::tuple<int, int> idx)
    {
        int i = std::get<0>(idx);
        int j = std::get<1>(idx);
        if (i < 0 || i > 2 || j < 0 || j > 3)
            throw py::index_error("Frame index out of range");

        return frm(i, j);
    });
    frame.def("__setitem__", [](KDL::Frame &frm, std::tuple<int, int> idx, double value)
    {
        int i = std::get<0>(idx);
        int j = std::get<1>(idx);
        if (i < 0 || i > 2 || j < 0 || j > 3)
            throw py::index_error("Frame index out of range");

        if (j == 3)
            frm.p(i) = value;
        else
            frm.M(i, j) = value;
    });
    frame.def("__repr__", [](const KDL::Frame &frm)
    {
        std::ostringstream oss;
        oss << frm;
        return oss.str();
    });
    frame.def("__copy__", [](const KDL::Frame& self)
    {
        return KDL::Frame(self);
    });
    frame.def("__deepcopy__", [](const KDL::Frame& self, py::dict)
    {
        return KDL::Frame(self);
    }, py::arg("memo"));
    frame.def("DH_Craig1989", &KDL::Frame::DH_Craig1989);
    frame.def("DH", &KDL::Frame::DH);
    frame.def("Inverse", (KDL::Frame (KDL::Frame::*)() const) &KDL::Frame::Inverse);
    frame.def("Inverse", (KDL::Vector (KDL::Frame::*)(const KDL::Vector&) const) &KDL::Frame::Inverse);
    frame.def("Inverse", (KDL::Wrench (KDL::Frame::*)(const KDL::Wrench&) const) &KDL::Frame::Inverse);
    frame.def("Inverse", (KDL::Twist (KDL::Frame::*)(const KDL::Twist&) const) &KDL::Frame::Inverse);
    frame.def_static("Identity", &KDL::Frame::Identity);
    frame.def("Integrate", &KDL::Frame::Integrate);
    frame.def("__mul__", (KDL::Vector (KDL::Frame::*)(const KDL::Vector&) const) &KDL::Frame::operator*, py::is_operator());
    frame.def("__mul__", (KDL::Wrench (KDL::Frame::*)(const KDL::Wrench&) const) &KDL::Frame::operator*, py::is_operator());
    frame.def("__mul__", (KDL::Twist (KDL::Frame::*)(const KDL::Twist&) const) &KDL::Frame::operator*, py::is_operator());

    frame.def("__mul__", [](const KDL::Frame &a, const KDL::Frame &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    frame.def("__eq__", [](const KDL::Frame &a, const KDL::Frame &b)
    {
        return operator==(a, b);
    }, py::is_operator());

    frame.def("__ne__", [](const KDL::Frame &a, const KDL::Frame &b)
    {
        return operator!=(a, b);
    }, py::is_operator());

    frame.def(py::pickle(
            [](const KDL::Frame &frm)
            { // __getstate__
                /* Return a tuple that fully encodes the state of the object */
                return py::make_tuple(frm.M, frm.p);
            },
            [](py::tuple t)
            { // __setstate__
                if (t.size() != 2)
                    throw std::runtime_error("Invalid state!");

                /* Create a new C++ instance */
                KDL::Frame frm(t[0].cast<KDL::Rotation>(), t[1].cast<KDL::Vector>());
                return frm;
            }));

    m.def("Equal", (bool (*)(const KDL::Frame&, const KDL::Frame&, double eps)) &KDL::Equal,
          py::arg("a"), py::arg("b"), py::arg("eps")=KDL::epsilon);


    // --------------------
    // Frame Global
    // --------------------
    m.def("diff", (KDL::Vector (*)(const KDL::Vector&, const KDL::Vector&, double dt)) &KDL::diff,
          py::arg("a"), py::arg("b"), py::arg("dt") = 1);
    m.def("diff", (KDL::Vector (*)(const KDL::Rotation&, const KDL::Rotation&, double dt)) &KDL::diff,
          py::arg("a"), py::arg("b"), py::arg("dt") = 1);
    m.def("diff", (KDL::Twist (*)(const KDL::Frame&, const KDL::Frame&, double dt)) &KDL::diff,
          py::arg("a"), py::arg("b"), py::arg("dt") = 1);
    m.def("diff", (KDL::Twist (*)(const KDL::Twist&, const KDL::Twist&, double dt)) &KDL::diff,
          py::arg("a"), py::arg("b"), py::arg("dt") = 1);
    m.def("diff", (KDL::Wrench (*)(const KDL::Wrench&, const KDL::Wrench&, double dt)) &KDL::diff,
          py::arg("a"), py::arg("b"), py::arg("dt") = 1);
    m.def("addDelta", (KDL::Vector (*)(const KDL::Vector&, const KDL::Vector&, double dt)) &KDL::addDelta,
          py::arg("a"), py::arg("da"), py::arg("dt") = 1);
    m.def("addDelta", (KDL::Rotation (*)(const KDL::Rotation&, const KDL::Vector&, double dt)) &KDL::addDelta,
          py::arg("a"), py::arg("da"), py::arg("dt") = 1);
    m.def("addDelta", (KDL::Frame (*)(const KDL::Frame&, const KDL::Twist&, double dt)) &KDL::addDelta,
          py::arg("a"), py::arg("da"), py::arg("dt") = 1);
    m.def("addDelta", (KDL::Twist (*)(const KDL::Twist&, const KDL::Twist&, double dt)) &KDL::addDelta,
          py::arg("a"), py::arg("da"), py::arg("dt") = 1);
    m.def("addDelta", (KDL::Wrench (*)(const KDL::Wrench&, const KDL::Wrench&, double dt)) &KDL::addDelta,
          py::arg("a"), py::arg("da"), py::arg("dt") = 1);

    // --------------------
    // Dynamics
    // --------------------

    // JntSpaceInertiaMatrix
    py::class_<KDL::JntSpaceInertiaMatrix> jnt_space_inertia_matrix(m, "JntSpaceInertiaMatrix");
    jnt_space_inertia_matrix.def(py::init<>());
    jnt_space_inertia_matrix.def(py::init<int>());
    jnt_space_inertia_matrix.def(py::init<const KDL::JntSpaceInertiaMatrix&>());
    jnt_space_inertia_matrix.def("resize", &KDL::JntSpaceInertiaMatrix::resize);
    jnt_space_inertia_matrix.def("rows", &KDL::JntSpaceInertiaMatrix::rows);
    jnt_space_inertia_matrix.def("columns", &KDL::JntSpaceInertiaMatrix::columns);
    jnt_space_inertia_matrix.def("__getitem__", [](const KDL::JntSpaceInertiaMatrix &jm, std::tuple<int, int> idx)
    {
        int i = std::get<0>(idx);
        int j = std::get<1>(idx);
        if (i < 0 || (unsigned int)i >= jm.rows() || j < 0 || (unsigned int)j >= jm.columns())
            throw py::index_error("Inertia index out of range");

        return jm((unsigned int)i, (unsigned int)j);
    });
    jnt_space_inertia_matrix.def("__setitem__", [](KDL::JntSpaceInertiaMatrix &jm, std::tuple<int, int> idx, double value)
    {
        int i = std::get<0>(idx);
        int j = std::get<1>(idx);
        if (i < 0 || (unsigned int)i >= jm.rows() || j < 0 || (unsigned int)j >= jm.columns())
            throw py::index_error("Inertia index out of range");

        jm((unsigned int)i, (unsigned int)j) = value;
    });
    jnt_space_inertia_matrix.def("__repr__", [](const KDL::JntSpaceInertiaMatrix &jm)
    {
        std::ostringstream oss;
        oss << jm;
        return oss.str();
    });
    jnt_space_inertia_matrix.def("__eq__", [](const KDL::JntSpaceInertiaMatrix &a, const KDL::JntSpaceInertiaMatrix &b)
    {
        return operator==(a, b);
    }, py::is_operator());


    m.def("Add", (void (*)(const KDL::JntSpaceInertiaMatrix&, const KDL::JntSpaceInertiaMatrix&, KDL::JntSpaceInertiaMatrix&)) &KDL::Add);
    m.def("Subtract", (void (*)(const KDL::JntSpaceInertiaMatrix&, const KDL::JntSpaceInertiaMatrix&, KDL::JntSpaceInertiaMatrix&)) &KDL::Subtract);
    m.def("Multiply", (void (*)(const KDL::JntSpaceInertiaMatrix&, const double&, KDL::JntSpaceInertiaMatrix&)) &KDL::Multiply);
    m.def("Divide", (void (*)(const KDL::JntSpaceInertiaMatrix&, const double&, KDL::JntSpaceInertiaMatrix&)) &KDL::Divide);
    m.def("Multiply", (void (*)(const KDL::JntSpaceInertiaMatrix&, const KDL::JntArray&, KDL::JntArray&)) &KDL::Multiply);
    m.def("SetToZero", (void (*)(KDL::JntSpaceInertiaMatrix&)) &KDL::SetToZero);
    m.def("Equal", (bool (*)(const KDL::JntSpaceInertiaMatrix&, const KDL::JntSpaceInertiaMatrix&, double)) &KDL::Equal,
          py::arg("src1"), py::arg("src2"), py::arg("eps")=KDL::epsilon);

    // ChainDynParam
    py::class_<KDL::ChainDynParam> chain_dyn_param(m, "ChainDynParam");
    chain_dyn_param.def(py::init<const KDL::Chain&, KDL::Vector>());
    chain_dyn_param.def("JntToCoriolis", &KDL::ChainDynParam::JntToCoriolis);
    chain_dyn_param.def("JntToMass", &KDL::ChainDynParam::JntToMass);
    chain_dyn_param.def("JntToGravity", &KDL::ChainDynParam::JntToGravity);

    // --------------------
    // FrameVel
    // --------------------

    // DoubleVel
    py::class_<KDL::doubleVel> double_vel(m, "doubleVel");
    double_vel.def(py::init<>());
    double_vel.def(py::init<const double>());
    double_vel.def(py::init<const double, const double>());
    double_vel.def(py::init<const KDL::doubleVel&>());
    double_vel.def_readwrite("t", &KDL::doubleVel::t);
    double_vel.def_readwrite("grad", &KDL::doubleVel::grad);
    double_vel.def("value", &KDL::doubleVel::value);
    double_vel.def("deriv", &KDL::doubleVel::deriv);
    double_vel.def("__repr__", [](const KDL::doubleVel &d)
    {
        std::ostringstream oss;
        oss << d;
        return oss.str();
    });
    double_vel.def("__copy__", [](const KDL::doubleVel& self)
    {
        return KDL::doubleVel(self);
    });
    double_vel.def("__deepcopy__", [](const KDL::doubleVel& self, py::dict)
    {
        return KDL::doubleVel(self);
    }, py::arg("memo"));

    m.def("diff", (KDL::doubleVel (*)(const KDL::doubleVel&, const KDL::doubleVel&, double)) &KDL::diff,
          py::arg("a"), py::arg("b"), py::arg("dt")=1.0);
    m.def("addDelta", (KDL::doubleVel (*)(const KDL::doubleVel&, const KDL::doubleVel&, double)) &KDL::addDelta,
          py::arg("a"), py::arg("da"), py::arg("dt")=1.0);
    m.def("Equal", (bool (*)(const KDL::doubleVel&, const KDL::doubleVel&, double)) &KDL::Equal,
          py::arg("r1"), py::arg("r2"), py::arg("eps")=KDL::epsilon);


    // VectorVel
    py::class_<KDL::VectorVel> vector_vel(m, "VectorVel");
    vector_vel.def_readwrite("p", &KDL::VectorVel::p);
    vector_vel.def_readwrite("v", &KDL::VectorVel::v);
    vector_vel.def(py::init<>());
    vector_vel.def(py::init<const KDL::Vector&, const KDL::Vector&>());
    vector_vel.def(py::init<const KDL::Vector&>());
    vector_vel.def(py::init<const KDL::VectorVel&>());
    vector_vel.def("value", &KDL::VectorVel::value);
    vector_vel.def("deriv", &KDL::VectorVel::deriv);
    vector_vel.def("__repr__", [](const KDL::VectorVel &vv)
    {
        std::ostringstream oss;
        oss << vv;
        return oss.str();
    });
    vector_vel.def("__copy__", [](const KDL::VectorVel& self)
    {
        return KDL::VectorVel(self);
    });
    vector_vel.def("__deepcopy__", [](const KDL::VectorVel& self, py::dict)
    {
        return KDL::VectorVel(self);
    }, py::arg("memo"));
    vector_vel.def_static("Zero", &KDL::VectorVel::Zero);
    vector_vel.def("ReverseSign", &KDL::VectorVel::ReverseSign);
    vector_vel.def("Norm", (KDL::doubleVel (KDL::VectorVel::*)() const) &KDL::VectorVel::Norm);


    vector_vel.def("__mul__", [](const KDL::VectorVel &a, const KDL::VectorVel &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    vector_vel.def("__mul__", [](const KDL::VectorVel &a, double &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    vector_vel.def("__mul__", [](double &a, const KDL::VectorVel &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    vector_vel.def("__mul__", [](const KDL::VectorVel &a, const KDL::Vector &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    vector_vel.def("__mul__", [](const KDL::Vector &a, const KDL::VectorVel &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    vector_vel.def("__mul__", [](const KDL::VectorVel &a, const KDL::doubleVel &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    vector_vel.def("__mul__", [](const KDL::doubleVel &a, const KDL::VectorVel &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    vector_vel.def("__mul__", [](const KDL::Rotation &a, const KDL::VectorVel &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    vector_vel.def("__iadd__", (KDL::VectorVel (KDL::VectorVel::*)(const KDL::VectorVel&) const) &KDL::VectorVel::operator+=, py::is_operator());
    vector_vel.def("__isub__", (KDL::VectorVel (KDL::VectorVel::*)(const KDL::VectorVel&) const) &KDL::VectorVel::operator-=, py::is_operator());

    vector_vel.def("__add__", [](const KDL::VectorVel &a, const KDL::VectorVel &b)
    {
        return operator+(a, b);
    }, py::is_operator());

    vector_vel.def("__add__", [](const KDL::Vector &a, const KDL::VectorVel &b)
    {
        return operator+(a, b);
    }, py::is_operator());

    vector_vel.def("__add__", [](const KDL::VectorVel &a, const KDL::Vector &b)
    {
        return operator+(a, b);
    }, py::is_operator());

    vector_vel.def("__sub__", [](const KDL::VectorVel &a, const KDL::VectorVel &b)
    {
        return operator-(a, b);
    }, py::is_operator());

    vector_vel.def("__sub__", [](const KDL::Vector &a, const KDL::VectorVel &b)
    {
        return operator-(a, b);
    }, py::is_operator());

    vector_vel.def("__sub__", [](const KDL::VectorVel &a, const KDL::Vector &b)
    {
        return operator-(a, b);
    }, py::is_operator());

    vector_vel.def("__truediv__", [](const KDL::VectorVel &a, double &b)
    {
        return operator/(a, b);
    }, py::is_operator());

    vector_vel.def("__truediv__", [](const KDL::VectorVel &a, const KDL::doubleVel &b)
    {
        return operator/(a, b);
    }, py::is_operator());

    vector_vel.def(py::pickle(
            [](const KDL::VectorVel &vv)
            { // __getstate__
                /* Return a tuple that fully encodes the state of the object */
                return py::make_tuple(vv.p, vv.v);
            },
            [](py::tuple t)
            { // __setstate__
                if (t.size() != 2)
                    throw std::runtime_error("Invalid state!");

                /* Create a new C++ instance */
                KDL::VectorVel vv(t[0].cast<KDL::Vector>(), t[1].cast<KDL::Vector>());
                return vv;
            }));

    m.def("SetToZero", (void (*)(KDL::VectorVel&)) &KDL::SetToZero);
    m.def("Equal", (bool (*)(const KDL::VectorVel&, const KDL::VectorVel&, double)) &KDL::Equal,
          py::arg("r1"), py::arg("r2"), py::arg("eps")=KDL::epsilon);
    m.def("Equal", (bool (*)(const KDL::Vector&, const KDL::VectorVel&, double)) &KDL::Equal,
          py::arg("r1"), py::arg("r2"), py::arg("eps")=KDL::epsilon);
    m.def("Equal", (bool (*)(const KDL::VectorVel&, const KDL::Vector&, double)) &KDL::Equal,
          py::arg("r1"), py::arg("r2"), py::arg("eps")=KDL::epsilon);

    m.def("dot", (KDL::doubleVel (*)(const KDL::VectorVel&, const KDL::VectorVel&)) &KDL::dot);
    m.def("dot", (KDL::doubleVel (*)(const KDL::VectorVel&, const KDL::Vector&)) &KDL::dot);
    m.def("dot", (KDL::doubleVel (*)(const KDL::Vector&, const KDL::VectorVel&)) &KDL::dot);

    // TwistVel
    py::class_<KDL::TwistVel> twist_vel(m, "TwistVel");
    twist_vel.def_readwrite("vel", &KDL::TwistVel::vel);
    twist_vel.def_readwrite("rot", &KDL::TwistVel::rot);
    twist_vel.def(py::init<>());
    twist_vel.def(py::init<const KDL::VectorVel&, const KDL::VectorVel&>());
    twist_vel.def(py::init<const KDL::Twist&, const KDL::Twist&>());
    twist_vel.def(py::init<const KDL::Twist&>());
    twist_vel.def(py::init<const KDL::TwistVel&>());
    twist_vel.def("value", &KDL::TwistVel::value);
    twist_vel.def("deriv", &KDL::TwistVel::deriv);
    twist_vel.def("__repr__", [](const KDL::TwistVel &tv)
    {
        std::ostringstream oss;
        oss << tv;
        return oss.str();
    });
    twist_vel.def("__copy__", [](const KDL::TwistVel& self)
    {
        return KDL::TwistVel(self);
    });
    twist_vel.def("__deepcopy__", [](const KDL::TwistVel& self, py::dict)
    {
        return KDL::TwistVel(self);
    }, py::arg("memo"));
    twist_vel.def_static("Zero", &KDL::TwistVel::Zero);
    twist_vel.def("ReverseSign", &KDL::TwistVel::ReverseSign);
    twist_vel.def("RefPoint", &KDL::TwistVel::RefPoint);
    twist_vel.def("GetTwist", &KDL::TwistVel::GetTwist);
    twist_vel.def("GetTwistDot", &KDL::TwistVel::GetTwistDot);

    twist_vel.def("__iadd__", (KDL::TwistVel (KDL::TwistVel::*)(const KDL::TwistVel&) const) &KDL::TwistVel::operator+=, py::is_operator());
    twist_vel.def("__isub__", (KDL::TwistVel (KDL::TwistVel::*)(const KDL::TwistVel&) const) &KDL::TwistVel::operator-=, py::is_operator());

    twist_vel.def("__mul__", [](const KDL::TwistVel &a, double &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    twist_vel.def("__mul__", [](double &a, const KDL::TwistVel &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    twist_vel.def("__mul__", [](const KDL::TwistVel &a, const KDL::doubleVel &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    twist_vel.def("__mul__", [](const KDL::doubleVel &a, const KDL::TwistVel &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    twist_vel.def("__truediv__", [](const KDL::TwistVel &a, double &b)
    {
        return operator/(a, b);
    }, py::is_operator());

    twist_vel.def("__truediv__", [](const KDL::TwistVel &a, const KDL::doubleVel &b)
    {
        return operator/(a, b);
    }, py::is_operator());

    twist_vel.def("__add__", [](const KDL::TwistVel &a, const KDL::TwistVel &b)
    {
        return operator+(a, b);
    }, py::is_operator());

    twist_vel.def("__sub__", [](const KDL::TwistVel &a, const KDL::TwistVel &b)
    {
        return operator-(a, b);
    }, py::is_operator());

    twist_vel.def("__neg__", [](const KDL::TwistVel &a)
    {
        return operator-(a);
    }, py::is_operator());
    twist_vel.def(py::pickle(
            [](const KDL::TwistVel &tv)
            { // __getstate__
                /* Return a tuple that fully encodes the state of the object */
                return py::make_tuple(tv.vel, tv.rot);
            },
            [](py::tuple t)
            { // __setstate__
                if (t.size() != 2)
                    throw std::runtime_error("Invalid state!");

                /* Create a new C++ instance */
                KDL::TwistVel tv(t[0].cast<KDL::VectorVel>(), t[1].cast<KDL::VectorVel>());
                return tv;
            }));

    m.def("SetToZero", (void (*)(KDL::TwistVel&)) &KDL::SetToZero);
    m.def("Equal", (bool (*)(const KDL::TwistVel&, const KDL::TwistVel&, double)) &KDL::Equal,
          py::arg("a"), py::arg("b"), py::arg("eps")=KDL::epsilon);
    m.def("Equal", (bool (*)(const KDL::Twist&, const KDL::TwistVel&, double)) &KDL::Equal,
          py::arg("a"), py::arg("b"), py::arg("eps")=KDL::epsilon);
    m.def("Equal", (bool (*)(const KDL::TwistVel&, const KDL::Twist&, double)) &KDL::Equal,
          py::arg("a"), py::arg("b"), py::arg("eps")=KDL::epsilon);

    // RotationVel
    py::class_<KDL::RotationVel> rotation_vel(m, "RotationVel");
    rotation_vel.def_readwrite("R", &KDL::RotationVel::R);
    rotation_vel.def_readwrite("w", &KDL::RotationVel::w);
    rotation_vel.def(py::init<>());
    rotation_vel.def(py::init<const KDL::Rotation&>());
    rotation_vel.def(py::init<const KDL::Rotation&, const KDL::Vector&>());
    rotation_vel.def(py::init<const KDL::RotationVel&>());
    rotation_vel.def("value", &KDL::RotationVel::value);
    rotation_vel.def("deriv", &KDL::RotationVel::deriv);
    rotation_vel.def("__repr__", [](const KDL::RotationVel &rv)
    {
        std::ostringstream oss;
        oss << rv;
        return oss.str();
    });
    rotation_vel.def("__copy__", [](const KDL::RotationVel& self)
    {
        return KDL::RotationVel(self);
    });
    rotation_vel.def("__deepcopy__", [](const KDL::RotationVel& self, py::dict)
    {
        return KDL::RotationVel(self);
    }, py::arg("memo"));
    rotation_vel.def("UnitX", &KDL::RotationVel::UnitX);
    rotation_vel.def("UnitY", &KDL::RotationVel::UnitY);
    rotation_vel.def("UnitZ", &KDL::RotationVel::UnitZ);
    rotation_vel.def_static("Identity", &KDL::RotationVel::Identity);
    rotation_vel.def("Inverse", (KDL::RotationVel (KDL::RotationVel::*)(void) const) &KDL::RotationVel::Inverse);
    rotation_vel.def("Inverse", (KDL::VectorVel (KDL::RotationVel::*)(const KDL::VectorVel&) const) &KDL::RotationVel::Inverse);
    rotation_vel.def("Inverse", (KDL::VectorVel (KDL::RotationVel::*)(const KDL::Vector&) const) &KDL::RotationVel::Inverse);
    rotation_vel.def("DoRotX", &KDL::RotationVel::DoRotX);
    rotation_vel.def("DoRotY", &KDL::RotationVel::DoRotY);
    rotation_vel.def("DoRotZ", &KDL::RotationVel::DoRotZ);
    rotation_vel.def_static("RotX", &KDL::RotationVel::RotX);
    rotation_vel.def_static("RotY", &KDL::RotationVel::RotY);
    rotation_vel.def_static("RotZ", &KDL::RotationVel::RotZ);
    rotation_vel.def_static("Rot", &KDL::RotationVel::Rot);
    rotation_vel.def_static("Rot2", &KDL::RotationVel::Rot2);

    rotation_vel.def("Inverse", (KDL::TwistVel (KDL::RotationVel::*)(const KDL::TwistVel&) const) &KDL::RotationVel::Inverse);
    rotation_vel.def("Inverse", (KDL::TwistVel (KDL::RotationVel::*)(const KDL::Twist&) const) &KDL::RotationVel::Inverse);

    rotation_vel.def("__mul__", [](const KDL::RotationVel &a, const KDL::RotationVel &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    rotation_vel.def("__mul__", [](const KDL::Rotation &a, const KDL::RotationVel &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    rotation_vel.def("__mul__", [](const KDL::RotationVel &a, const KDL::Rotation &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    rotation_vel.def("__mul__", (KDL::VectorVel (KDL::RotationVel::*)(const KDL::VectorVel&) const) &KDL::RotationVel::operator*, py::is_operator());
    rotation_vel.def("__mul__", (KDL::VectorVel (KDL::RotationVel::*)(const KDL::Vector&) const) &KDL::RotationVel::operator*, py::is_operator());

    rotation_vel.def("__mul__", (KDL::TwistVel (KDL::RotationVel::*)(const KDL::TwistVel&) const) &KDL::RotationVel::operator*, py::is_operator());
    rotation_vel.def("__mul__", (KDL::TwistVel (KDL::RotationVel::*)(const KDL::Twist&) const) &KDL::RotationVel::operator*, py::is_operator());


    rotation_vel.def(py::pickle(
            [](const KDL::RotationVel &rv)
            { // __getstate__
                /* Return a tuple that fully encodes the state of the object */
                return py::make_tuple(rv.R, rv.w);
            },
            [](py::tuple t)
            { // __setstate__
                if (t.size() != 2)
                    throw std::runtime_error("Invalid state!");

                /* Create a new C++ instance */
                KDL::RotationVel rv(t[0].cast<KDL::Rotation>(), t[1].cast<KDL::Vector>());
                return rv;
            }));

    m.def("Equal", (bool (*)(const KDL::RotationVel&, const KDL::RotationVel&, double)) &KDL::Equal,
          py::arg("r1"), py::arg("r2"), py::arg("eps")=KDL::epsilon);
    m.def("Equal", (bool (*)(const KDL::Rotation&, const KDL::RotationVel&, double)) &KDL::Equal,
          py::arg("r1"), py::arg("r2"), py::arg("eps")=KDL::epsilon);
    m.def("Equal", (bool (*)(const KDL::RotationVel&, const KDL::Rotation&, double)) &KDL::Equal,
          py::arg("r1"), py::arg("r2"), py::arg("eps")=KDL::epsilon);


    // FrameVel
    py::class_<KDL::FrameVel> frame_vel(m, "FrameVel");
    frame_vel.def_readwrite("M", &KDL::FrameVel::M);
    frame_vel.def_readwrite("p", &KDL::FrameVel::p);
    frame_vel.def(py::init<>());
    frame_vel.def(py::init<const KDL::Frame&>());
    frame_vel.def(py::init<const KDL::Frame&, const KDL::Twist&>());
    frame_vel.def(py::init<const KDL::RotationVel&, const KDL::VectorVel&>());
    frame_vel.def(py::init<const KDL::FrameVel&>());
    frame_vel.def("value", &KDL::FrameVel::value);
    frame_vel.def("deriv", &KDL::FrameVel::deriv);
    frame_vel.def("__repr__", [](const KDL::FrameVel &fv)
    {
        std::ostringstream oss;
        oss << fv;
        return oss.str();
    });
    frame_vel.def("__copy__", [](const KDL::FrameVel& self)
    {
        return KDL::FrameVel(self);
    });
    frame_vel.def("__deepcopy__", [](const KDL::FrameVel& self, py::dict)
    {
        return KDL::FrameVel(self);
    }, py::arg("memo"));
    frame_vel.def_static("Identity", &KDL::FrameVel::Identity);
    frame_vel.def("Inverse", (KDL::FrameVel (KDL::FrameVel::*)() const) &KDL::FrameVel::Inverse);
    frame_vel.def("Inverse", (KDL::VectorVel (KDL::FrameVel::*)(const KDL::VectorVel&) const) &KDL::FrameVel::Inverse);
    frame_vel.def("Inverse", (KDL::VectorVel (KDL::FrameVel::*)(const KDL::Vector&) const) &KDL::FrameVel::Inverse);
    
    frame_vel.def("__mul__", [](const KDL::FrameVel &a, const KDL::FrameVel &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    frame_vel.def("__mul__", [](const KDL::Frame &a, const KDL::FrameVel &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    frame_vel.def("__mul__", [](const KDL::FrameVel &a, const KDL::Frame &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    frame_vel.def("__mul__", (KDL::VectorVel (KDL::FrameVel::*)(const KDL::VectorVel&) const) &KDL::FrameVel::operator*, py::is_operator());
    frame_vel.def("__mul__", (KDL::VectorVel (KDL::FrameVel::*)(const KDL::Vector&) const) &KDL::FrameVel::operator*, py::is_operator());

    frame_vel.def("__mul__", (KDL::TwistVel (KDL::FrameVel::*)(const KDL::TwistVel&) const) &KDL::FrameVel::operator*, py::is_operator());
    frame_vel.def("__mul__", (KDL::TwistVel (KDL::FrameVel::*)(const KDL::Twist&) const) &KDL::FrameVel::operator*, py::is_operator());

    frame_vel.def("GetFrame", &KDL::FrameVel::GetFrame);
    frame_vel.def("GetTwist", &KDL::FrameVel::GetTwist);
    frame_vel.def("Inverse", (KDL::TwistVel (KDL::FrameVel::*)(const KDL::TwistVel&) const) &KDL::FrameVel::Inverse);
    frame_vel.def("Inverse", (KDL::TwistVel (KDL::FrameVel::*)(const KDL::Twist&) const) &KDL::FrameVel::Inverse);

    frame_vel.def(py::pickle(
            [](const KDL::FrameVel &fv)
            { // __getstate__
                /* Return a tuple that fully encodes the state of the object */
                return py::make_tuple(fv.M, fv.p);
            },
            [](py::tuple t)
            { // __setstate__
                if (t.size() != 2)
                    throw std::runtime_error("Invalid state!");

                /* Create a new C++ instance */
                KDL::FrameVel rv(t[0].cast<KDL::RotationVel>(), t[1].cast<KDL::VectorVel>());
                return rv;
            }));

    m.def("Equal", (bool (*)(const KDL::FrameVel&, const KDL::FrameVel&, double)) &KDL::Equal,
          py::arg("r1"), py::arg("r2"), py::arg("eps")=KDL::epsilon);
    m.def("Equal", (bool (*)(const KDL::Frame&, const KDL::FrameVel&, double)) &KDL::Equal,
          py::arg("r1"), py::arg("r2"), py::arg("eps")=KDL::epsilon);
    m.def("Equal", (bool (*)(const KDL::FrameVel&, const KDL::Frame&, double)) &KDL::Equal,
          py::arg("r1"), py::arg("r2"), py::arg("eps")=KDL::epsilon);


    // --------------------
    // FrameVel Global
    // --------------------
    m.def("diff", (KDL::VectorVel (*)(const KDL::VectorVel&, const KDL::VectorVel&, double)) &KDL::diff,
          py::arg("a"), py::arg("b"), py::arg("dt")=1.0);
    m.def("diff", (KDL::VectorVel (*)(const KDL::RotationVel&, const KDL::RotationVel&, double)) &KDL::diff,
          py::arg("a"), py::arg("b"), py::arg("dt")=1.0);
    m.def("diff", (KDL::TwistVel (*)(const KDL::FrameVel&, const KDL::FrameVel&, double)) &KDL::diff,
          py::arg("a"), py::arg("b"), py::arg("dt")=1.0);

    m.def("addDelta", (KDL::VectorVel (*)(const KDL::VectorVel&, const KDL::VectorVel&, double)) &KDL::addDelta,
          py::arg("a"), py::arg("da"), py::arg("dt")=1.0);
    m.def("addDelta", (KDL::RotationVel (*)(const KDL::RotationVel&, const KDL::VectorVel&, double)) &KDL::addDelta,
          py::arg("a"), py::arg("da"), py::arg("dt")=1.0);
    m.def("addDelta", (KDL::FrameVel (*)(const KDL::FrameVel&, const KDL::TwistVel&, double)) &KDL::addDelta,
          py::arg("a"), py::arg("da"), py::arg("dt")=1.0);

    // --------------------
    // Kinfam
    // --------------------

    // Joint
    py::class_<KDL::Joint> joint(m, "Joint");
    py::enum_<KDL::Joint::JointType> joint_type(joint, "JointType");
        joint_type.value("RotAxis", KDL::Joint::JointType::RotAxis);
        joint_type.value("RotX", KDL::Joint::JointType::RotX);
        joint_type.value("RotY", KDL::Joint::JointType::RotY);
        joint_type.value("RotZ", KDL::Joint::JointType::RotZ);
        joint_type.value("TransAxis", KDL::Joint::JointType::TransAxis);
        joint_type.value("TransX", KDL::Joint::JointType::TransX);
        joint_type.value("TransY", KDL::Joint::JointType::TransY);
        joint_type.value("TransZ", KDL::Joint::JointType::TransZ);
        joint_type.value("None", KDL::Joint::JointType::None);

        joint_type.export_values();

    joint.def(py::init<>());
    joint.def(py::init<std::string, KDL::Joint::JointType, double, double, double, double, double>(),
              py::arg("name"), py::arg("type")=KDL::Joint::JointType::None, py::arg("scale")=1, py::arg("offset")=0,
              py::arg("inertia")=0, py::arg("damping")=0, py::arg("stiffness")=0);
    joint.def(py::init<KDL::Joint::JointType, double, double, double, double, double>(),
              py::arg("type")=KDL::Joint::JointType::None, py::arg("scale")=1, py::arg("offset")=0,
              py::arg("inertia")=0, py::arg("damping")=0, py::arg("stiffness")=0);
    joint.def(py::init<std::string, KDL::Vector, KDL::Vector, KDL::Joint::JointType, double, double, double, double, double>(),
              py::arg("name"), py::arg("origin"), py::arg("axis"), py::arg("type"), py::arg("scale")=1, py::arg("offset")=0,
              py::arg("inertia")=0, py::arg("damping")=0, py::arg("stiffness")=0);
    joint.def(py::init<KDL::Vector, KDL::Vector, KDL::Joint::JointType, double, double, double, double, double>(),
              py::arg("origin"), py::arg("axis"), py::arg("type"), py::arg("scale")=1, py::arg("offset")=0,
              py::arg("inertia")=0, py::arg("damping")=0, py::arg("stiffness")=0);
    joint.def(py::init<const KDL::Joint&>());
    joint.def("pose", &KDL::Joint::pose);
    joint.def("twist", &KDL::Joint::twist);
    joint.def("JointAxis", &KDL::Joint::JointAxis);
    joint.def("JointOrigin", &KDL::Joint::JointOrigin);
    joint.def("getName", &KDL::Joint::getName);
    joint.def("getType", &KDL::Joint::getType);
    joint.def("getTypeName", &KDL::Joint::getTypeName);
    joint.def("__repr__", [](const KDL::Joint &j)
    {
        std::ostringstream oss;
        oss << j;
        return oss.str();
    });

    // RotationalInertia
    py::class_<KDL::RotationalInertia> rotational_inertia(m, "RotationalInertia");
    rotational_inertia.def(py::init<double, double, double, double, double, double>(),
                           py::arg("Ixx")=0, py::arg("Iyy")=0, py::arg("Izz")=0,
                           py::arg("Ixy")=0, py::arg("Ixz")=0, py::arg("Iyz")=0);
    rotational_inertia.def_static("Zero", &KDL::RotationalInertia::Zero);
    rotational_inertia.def("__getitem__", [](const KDL::RotationalInertia &inertia, int i)
    {
        if (i < 0 || i > 8)
            throw py::index_error("RotationalInertia index out of range");

        return inertia.data[i];
    });
    rotational_inertia.def("__setitem__", [](KDL::RotationalInertia &inertia, int i, double value)
    {
        if (i < 0 || i > 8)
            throw py::index_error("RotationalInertia index out of range");

        inertia.data[i] = value;
    });

    rotational_inertia.def("__mul__", [](double a, const KDL::RotationalInertia &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    rotational_inertia.def("__add__", [](const KDL::RotationalInertia &a, const KDL::RotationalInertia &b)
    {
        return operator+(a, b);
    }, py::is_operator());

    rotational_inertia.def("__mul__", (KDL::Vector (KDL::RotationalInertia::*)(const KDL::Vector&) const) &KDL::RotationalInertia::operator*, py::is_operator());

    // RigidBodyInertia
    py::class_<KDL::RigidBodyInertia> rigid_body_inertia(m, "RigidBodyInertia");
    rigid_body_inertia.def(py::init<double, const KDL::Vector&, const KDL::RotationalInertia&>(),
                           py::arg("m")=0, py::arg_v("oc", KDL::Vector::Zero(), "Vector.Zero"),
                           py::arg_v("Ic", KDL::RotationalInertia::Zero(), "RigidBodyInertia.Zero"));
    rigid_body_inertia.def_static("Zero", &KDL::RigidBodyInertia::Zero);
    rigid_body_inertia.def("RefPoint", &KDL::RigidBodyInertia::RefPoint);
    rigid_body_inertia.def("getMass", &KDL::RigidBodyInertia::getMass);
    rigid_body_inertia.def("getCOG", &KDL::RigidBodyInertia::getCOG);
    rigid_body_inertia.def("getRotationalInertia", &KDL::RigidBodyInertia::getRotationalInertia);

    rigid_body_inertia.def("__mul__", [](double a, const KDL::RigidBodyInertia &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    rigid_body_inertia.def("__add__", [](const KDL::RigidBodyInertia &a, const KDL::RigidBodyInertia &b)
    {
        return operator+(a, b);
    }, py::is_operator());

    rigid_body_inertia.def("__mul__", [](const KDL::RigidBodyInertia &a, const KDL::Twist &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    rigid_body_inertia.def("__mul__", [](const KDL::Frame &a, const KDL::RigidBodyInertia &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    rigid_body_inertia.def("__mul__", [](const KDL::Rotation &a, const KDL::RigidBodyInertia &b)
    {
        return operator*(a, b);
    }, py::is_operator());

    // Segment
    py::class_<KDL::Segment> segment(m, "Segment");
    segment.def(py::init<const std::string&, const KDL::Joint&, const KDL::Frame&, const KDL::RigidBodyInertia&>(),
                py::arg("name"), py::arg_v("joint", KDL::Joint(), "Joint"), py::arg_v("f_tip", KDL::Frame::Identity(), "Frame.Identity"),
                py::arg_v("I", KDL::RigidBodyInertia::Zero(), "RigidBodyInertia.Zero"));
    segment.def(py::init<const KDL::Joint&, const KDL::Frame&, const KDL::RigidBodyInertia&>(),
                py::arg_v("joint", KDL::Joint(), "Joint"), py::arg_v("f_tip", KDL::Frame::Identity(), "Frame.Identity"),
                py::arg_v("I", KDL::RigidBodyInertia::Zero(), "RigidBodyInertia.Zero"));
    segment.def(py::init<const KDL::Segment&>());
    segment.def("getFrameToTip", &KDL::Segment::getFrameToTip);
    segment.def("pose", &KDL::Segment::pose);
    segment.def("twist", &KDL::Segment::twist);
    segment.def("getName", &KDL::Segment::getName);
    segment.def("getJoint", &KDL::Segment::getJoint);
    segment.def("getInertia", &KDL::Segment::getInertia);
    segment.def("setInertia", &KDL::Segment::setInertia);

    // Chain
    py::class_<KDL::Chain> chain(m, "Chain");
    chain.def(py::init<>());
    chain.def(py::init<const KDL::Chain&>());
    chain.def("addSegment", &KDL::Chain::addSegment);
    chain.def("addChain", &KDL::Chain::addChain);
    chain.def("getNrOfJoints", &KDL::Chain::getNrOfJoints);
    chain.def("getNrOfSegments", &KDL::Chain::getNrOfSegments);
    chain.def("getSegment", (KDL::Segment& (KDL::Chain::*)(unsigned int)) &KDL::Chain::getSegment);
    chain.def("getSegment", (const KDL::Segment& (KDL::Chain::*)(unsigned int) const) &KDL::Chain::getSegment);
    chain.def("__repr__", [](const KDL::Chain &c)
    {
        std::ostringstream oss;
        oss << c;
        return oss.str();
    });

    // Tree
    py::class_<KDL::Tree> tree(m, "Tree");
    tree.def(py::init<const std::string&>(), py::arg("root_name")="root");
    tree.def("addSegment", &KDL::Tree::addSegment);
    tree.def("addChain", &KDL::Tree::addChain);
    tree.def("addTree", &KDL::Tree::addTree);
    tree.def("getNrOfJoints", &KDL::Tree::getNrOfJoints);
    tree.def("getNrOfSegments", &KDL::Tree::getNrOfSegments);
    tree.def("getChain", [](const KDL::Tree &tree, const std::string& chain_root, const std::string& chain_tip)
    {
        KDL::Chain* chain = new KDL::Chain();
        tree.getChain(chain_root, chain_tip, *chain);
        return chain;
    });
    tree.def("__repr__", [](const KDL::Tree &t)
    {
        std::ostringstream oss;
        oss << t;
        return oss.str();
    });

    // Jacobian
    py::class_<KDL::Jacobian> jacobian(m, "Jacobian");
    jacobian.def(py::init<>());
    jacobian.def(py::init<unsigned int>());
    jacobian.def(py::init<const KDL::Jacobian&>());
    jacobian.def("rows", &KDL::Jacobian::rows);
    jacobian.def("columns", &KDL::Jacobian::columns);
    jacobian.def("resize", &KDL::Jacobian::resize);
    jacobian.def("getColumn", &KDL::Jacobian::getColumn);
    jacobian.def("setColumn", &KDL::Jacobian::setColumn);
    jacobian.def("changeRefPoint", &KDL::Jacobian::changeRefPoint);
    jacobian.def("changeBase", &KDL::Jacobian::changeBase);
    jacobian.def("changeRefFrame", &KDL::Jacobian::changeRefFrame);
    jacobian.def("__getitem__", [](const KDL::Jacobian &jac, std::tuple<int, int> idx)
    {
        int i = std::get<0>(idx);
        int j = std::get<1>(idx);
        if (i < 0 || i > 5 || j < 0 || (unsigned int)j >= jac.columns())
            throw py::index_error("Jacobian index out of range");
        return jac((unsigned int)i, (unsigned int)j);
    });
    jacobian.def("__setitem__", [](KDL::Jacobian &jac, std::tuple<int, int> idx, double value)
    {
        int i = std::get<0>(idx);
        int j = std::get<1>(idx);
        if (i < 0 || i > 5 || j < 0 || (unsigned int)j >= jac.columns())
            throw py::index_error("Jacobian index out of range");

        jac((unsigned int)i, (unsigned int)j) = value;
    });
    jacobian.def("__repr__", [](const KDL::Jacobian &jac)
    {
        std::ostringstream oss;
        oss << jac;
        return oss.str();
    });

    m.def("SetToZero", (void (*)(KDL::Jacobian&)) &KDL::SetToZero);
    m.def("changeRefPoint", (void (*)(const KDL::Jacobian&, const KDL::Vector&, KDL::Jacobian&)) &KDL::changeRefPoint);
    m.def("changeBase", (void (*)(const KDL::Jacobian&, const KDL::Rotation&, KDL::Jacobian&)) &KDL::changeBase);
    m.def("SetToZero", (void (*)(const KDL::Jacobian&, const KDL::Frame&, KDL::Jacobian&)) &KDL::changeRefFrame);

    // JntArray
    py::class_<KDL::JntArray> jnt_array(m, "JntArray");
    jnt_array.def(py::init<>());
    jnt_array.def(py::init<unsigned int>());
    jnt_array.def(py::init<const KDL::JntArray&>());
    jnt_array.def("rows", &KDL::JntArray::rows);
    jnt_array.def("columns", &KDL::JntArray::columns);
    jnt_array.def("resize", &KDL::JntArray::resize);
    jnt_array.def("__getitem__", [](const KDL::JntArray &ja, int i)
    {
        if (i < 0 || (unsigned int)i >= ja.rows())
            throw py::index_error("JntArray index out of range");

        return ja(i);
    });
    jnt_array.def("__setitem__", [](KDL::JntArray &ja, int i, double value)
    {
        if (i < 0 || (unsigned int)i >= ja.rows())
            throw py::index_error("JntArray index out of range");

        ja(i) = value;
    });
    jnt_array.def("__repr__", [](const KDL::JntArray &ja)
    {
        std::ostringstream oss;
        oss << ja;
        return oss.str();
    });
    jnt_array.def("__eq__", [](const KDL::JntArray &a, const KDL::JntArray &b)
    {
        return operator==(a, b);
    }, py::is_operator());
    m.def("Add", (void (*)(const KDL::JntArray&, const KDL::JntArray&, KDL::JntArray&)) &KDL::Add);
    m.def("Subtract", (void (*)(const KDL::JntArray&, const KDL::JntArray&, KDL::JntArray&)) &KDL::Subtract);
    m.def("Multiply", (void (*)(const KDL::JntArray&, const double&, KDL::JntArray&)) &KDL::Multiply);
    m.def("Divide", (void (*)(const KDL::JntArray&, const double&, KDL::JntArray&)) &KDL::Divide);
    m.def("MultiplyJacobian", (void (*)(const KDL::Jacobian&, const KDL::JntArray&, KDL::Twist&)) &KDL::MultiplyJacobian);
    m.def("SetToZero", (void (*)(KDL::JntArray&)) &KDL::SetToZero);
    m.def("Equal", (bool (*)(const KDL::JntArray&, const KDL::JntArray&, double)) &KDL::Equal,
          py::arg("src1"), py::arg("src2"), py::arg("eps")=KDL::epsilon);

    // JntArrayVel
    py::class_<KDL::JntArrayVel> jnt_array_vel(m, "JntArrayVel");
    jnt_array_vel.def_readwrite("q", &KDL::JntArrayVel::q);
    jnt_array_vel.def_readwrite("qdot", &KDL::JntArrayVel::qdot);
    jnt_array_vel.def(py::init<unsigned int>());
    jnt_array_vel.def(py::init<const KDL::JntArray&, const KDL::JntArray&>());
    jnt_array_vel.def(py::init<const KDL::JntArray&>());
    jnt_array_vel.def("resize", &KDL::JntArrayVel::resize);
    jnt_array_vel.def("value", &KDL::JntArrayVel::value);
    jnt_array_vel.def("deriv", &KDL::JntArrayVel::deriv);

    m.def("Add", (void (*)(const KDL::JntArrayVel&, const KDL::JntArrayVel&, KDL::JntArrayVel&)) &KDL::Add);
    m.def("Add", (void (*)(const KDL::JntArrayVel&, const KDL::JntArray&, KDL::JntArrayVel&)) &KDL::Add);
    m.def("Subtract", (void (*)(const KDL::JntArrayVel&, const KDL::JntArrayVel&, KDL::JntArrayVel&)) &KDL::Subtract);
    m.def("Subtract", (void (*)(const KDL::JntArrayVel&, const KDL::JntArray&, KDL::JntArrayVel&)) &KDL::Subtract);
    m.def("Multiply", (void (*)(const KDL::JntArrayVel&, const double&, KDL::JntArrayVel&)) &KDL::Multiply);
    m.def("Multiply", (void (*)(const KDL::JntArrayVel&, const KDL::doubleVel&, KDL::JntArrayVel&)) &KDL::Multiply);
    m.def("Divide", (void (*)(const KDL::JntArrayVel&, const double&, KDL::JntArrayVel&)) &KDL::Divide);
    m.def("Divide", (void (*)(const KDL::JntArrayVel&, const KDL::doubleVel&, KDL::JntArrayVel&)) &KDL::Divide);
    m.def("SetToZero", (void (*)(KDL::JntArrayVel&)) &KDL::SetToZero);
    m.def("Equal", (bool (*)(const KDL::JntArrayVel&, const KDL::JntArrayVel&, double)) &KDL::Equal,
          py::arg("src1"), py::arg("src2"), py::arg("eps")=KDL::epsilon);

    // SolverI
    py::class_<KDL::SolverI> solver_i(m, "SolverI");
    solver_i.def("getError", &KDL::SolverI::getError);
    solver_i.def("strError", &KDL::SolverI::strError);
    solver_i.def("updateInternalDataStructures", &KDL::SolverI::updateInternalDataStructures);

    // ChainFkSolverPos
    py::class_<KDL::ChainFkSolverPos, KDL::SolverI> chain_fk_solver_pos(m, "ChainFkSolverPos");
    chain_fk_solver_pos.def("JntToCart", (int (KDL::ChainFkSolverPos::*)(const KDL::JntArray&, KDL::Frame&, int)) &KDL::ChainFkSolverPos::JntToCart,
                            py::arg("q_in"), py::arg("p_out"), py::arg("segmentNr")=-1);
    chain_fk_solver_pos.def("JntToCart", (int (KDL::ChainFkSolverPos::*)(const KDL::JntArray&, std::vector<KDL::Frame>&, int)) &KDL::ChainFkSolverPos::JntToCart,
                            py::arg("q_in"), py::arg("p_out"), py::arg("segmentNr")=-1);

    // ChainFkSolverVel
    py::class_<KDL::ChainFkSolverVel, KDL::SolverI> chain_fk_solver_vel(m, "ChainFkSolverVel");
    chain_fk_solver_vel.def("JntToCart", (int (KDL::ChainFkSolverVel::*)(const KDL::JntArrayVel&, KDL::FrameVel&, int)) &KDL::ChainFkSolverVel::JntToCart,
                            py::arg("q_in"), py::arg("p_out"), py::arg("segmentNr")=-1);

    // ChainFkSolverPos_recursive
    py::class_<KDL::ChainFkSolverPos_recursive, KDL::ChainFkSolverPos> chain_fk_solver_pos_recursive(m, "ChainFkSolverPos_recursive");
    chain_fk_solver_pos_recursive.def(py::init<const KDL::Chain&>());

    // ChainFkSolverVel_recursive
    py::class_<KDL::ChainFkSolverVel_recursive, KDL::ChainFkSolverVel> chain_fk_solver_vel_recursive(m, "ChainFkSolverVel_recursive");
    chain_fk_solver_vel_recursive.def(py::init<const KDL::Chain&>());

    // ChainIkSolverPos
    py::class_<KDL::ChainIkSolverPos, KDL::SolverI> chain_ik_solver_pos(m, "ChainIkSolverPos");
    chain_ik_solver_pos.def("CartToJnt", (int (KDL::ChainIkSolverPos::*)(const KDL::JntArray&, const KDL::Frame&, KDL::JntArray&)) &KDL::ChainIkSolverPos::CartToJnt,
                            py::arg("q_init"), py::arg("p_in"), py::arg("q_out"));

    // ChainIkSolverVel
    py::class_<KDL::ChainIkSolverVel, KDL::SolverI> chain_ik_solver_vel(m, "ChainIkSolverVel");
    chain_ik_solver_vel.def("CartToJnt", (int (KDL::ChainIkSolverVel::*)(const KDL::JntArray&, const KDL::Twist&, KDL::JntArray&)) &KDL::ChainIkSolverVel::CartToJnt,
                            py::arg("q_in"), py::arg("v_in"), py::arg("qdot_out"));

    // ChainIkSolverPos_NR
    py::class_<KDL::ChainIkSolverPos_NR, KDL::ChainIkSolverPos> chain_ik_solver_pos_NR(m, "ChainIkSolverPos_NR");
    chain_ik_solver_pos_NR.def(py::init<const KDL::Chain&, KDL::ChainFkSolverPos&, KDL::ChainIkSolverVel&, unsigned int, double>(),
                               py::arg("chain"), py::arg("fksolver"), py::arg("iksolver"),
                               py::arg("maxiter")=100, py::arg("eps")=KDL::epsilon);

    // ChainIkSolverPos_NR_JL
    py::class_<KDL::ChainIkSolverPos_NR_JL, KDL::ChainIkSolverPos> chain_ik_solver_pos_NR_JL(m, "ChainIkSolverPos_NR_JL");
    chain_ik_solver_pos_NR_JL.def(py::init<const KDL::Chain&, const KDL::JntArray&, const KDL::JntArray&, KDL::ChainFkSolverPos&,
                                  KDL::ChainIkSolverVel&, unsigned int, double>(),
                                  py::arg("chain"), py::arg("q_min"), py::arg("q_max"), py::arg("fksolver"),
                                  py::arg("iksolver"), py::arg("maxiter")=100, py::arg("eps")=KDL::epsilon);

    // ChainIkSolverVel_pinv
    py::class_<KDL::ChainIkSolverVel_pinv, KDL::ChainIkSolverVel> chain_ik_solver_vel_pinv(m, "ChainIkSolverVel_pinv");
    chain_ik_solver_vel_pinv.def(py::init<const KDL::Chain&, double, int>(),
                                 py::arg("chain"), py::arg("eps")=0.00001, py::arg("maxiter")=150);

    // ChainIkSolverVel_wdls
    py::class_<KDL::ChainIkSolverVel_wdls, KDL::ChainIkSolverVel> chain_ik_solver_vel_wdls(m, "ChainIkSolverVel_wdls");
    chain_ik_solver_vel_wdls.def(py::init<const KDL::Chain&, double, int>(),
                                 py::arg("chain"), py::arg("eps")=0.00001, py::arg("maxiter")=150);
    chain_ik_solver_vel_wdls.def("setWeightTS", &KDL::ChainIkSolverVel_wdls::setWeightTS);
    chain_ik_solver_vel_wdls.def("setWeightJS", &KDL::ChainIkSolverVel_wdls::setWeightJS);
    chain_ik_solver_vel_wdls.def("setLambda", &KDL::ChainIkSolverVel_wdls::setLambda);
    chain_ik_solver_vel_wdls.def("setEps", &KDL::ChainIkSolverVel_wdls::setEps);
    chain_ik_solver_vel_wdls.def("setMaxIter", &KDL::ChainIkSolverVel_wdls::setMaxIter);
    chain_ik_solver_vel_wdls.def("getNrZeroSigmas", &KDL::ChainIkSolverVel_wdls::getNrZeroSigmas);
    chain_ik_solver_vel_wdls.def("getSigmaMin", &KDL::ChainIkSolverVel_wdls::getSigmaMin);
    chain_ik_solver_vel_wdls.def("getSigma", &KDL::ChainIkSolverVel_wdls::getSigma);
    chain_ik_solver_vel_wdls.def("getEps", &KDL::ChainIkSolverVel_wdls::getEps);
    chain_ik_solver_vel_wdls.def("getLambda", &KDL::ChainIkSolverVel_wdls::getLambda);
    chain_ik_solver_vel_wdls.def("getLambdaScaled", &KDL::ChainIkSolverVel_wdls::getLambdaScaled);
    chain_ik_solver_vel_wdls.def("getSVDResult", &KDL::ChainIkSolverVel_wdls::getSVDResult);

    // ChainIkSolverPos_LMA
    py::class_<KDL::ChainIkSolverPos_LMA, KDL::ChainIkSolverPos> chain_ik_solver_pos_LMA(m, "ChainIkSolverPos_LMA");
    chain_ik_solver_pos_LMA.def(py::init<const KDL::Chain&, double, int, double>(),
                                py::arg("chain"), py::arg("eps")=1e-5, py::arg("maxiter")=500,
                                py::arg("eps_joints")=1e-15);

    // ChainIkSolverVel_pinv_nso
    py::class_<KDL::ChainIkSolverVel_pinv_nso, KDL::ChainIkSolverVel> chain_ik_solver_vel_pinv_nso(m, "ChainIkSolverVel_pinv_nso");
    chain_ik_solver_vel_pinv_nso.def(py::init<const KDL::Chain&, double, int, double>(),
                                     py::arg("chain"), py::arg("eps")=0.00001, py::arg("maxiter")=150, py::arg("alpha")=0.25);
    chain_ik_solver_vel_pinv_nso.def("setWeights", &KDL::ChainIkSolverVel_pinv_nso::setWeights);
    chain_ik_solver_vel_pinv_nso.def("setOptPos", &KDL::ChainIkSolverVel_pinv_nso::setOptPos);
    chain_ik_solver_vel_pinv_nso.def("setAlpha", &KDL::ChainIkSolverVel_pinv_nso::setAlpha);
    chain_ik_solver_vel_pinv_nso.def("getWeights", &KDL::ChainIkSolverVel_pinv_nso::getWeights);
    chain_ik_solver_vel_pinv_nso.def("getOptPos", &KDL::ChainIkSolverVel_pinv_nso::getOptPos);
    chain_ik_solver_vel_pinv_nso.def("getAlpha", &KDL::ChainIkSolverVel_pinv_nso::getAlpha);

    // ChainIkSolverVel_pinv_givens
    py::class_<KDL::ChainIkSolverVel_pinv_givens, KDL::ChainIkSolverVel> chain_ik_solver_vel_pinv_givens(m, "ChainIkSolverVel_pinv_givens");
    chain_ik_solver_vel_pinv_givens.def(py::init<const KDL::Chain&>());

    // ChainJntToJacSolver
    py::class_<KDL::ChainJntToJacSolver, KDL::SolverI> chain_jnt_to_jac_solver(m, "ChainJntToJacSolver");
    chain_jnt_to_jac_solver.def(py::init<const KDL::Chain&>());
    chain_jnt_to_jac_solver.def("JntToJac", &KDL::ChainJntToJacSolver::JntToJac,
                                py::arg("q_in"), py::arg("jac"), py::arg("seg_nr")=-1);
    chain_jnt_to_jac_solver.def("setLockedJoints", &KDL::ChainJntToJacSolver::setLockedJoints);

    // // ChainJntToJacDotSolver
    // py::class_<KDL::ChainJntToJacDotSolver, KDL::SolverI> chain_jnt_to_jac_dot_solver(m, "ChainJntToJacDotSolver");
    // chain_jnt_to_jac_dot_solver.def(py::init<const KDL::Chain&>());
    // chain_jnt_to_jac_dot_solver.def("JntToJacDot", (int (KDL::ChainJntToJacDotSolver::*)(const KDL::JntArrayVel&, KDL::Jacobian&, int)) &KDL::ChainJntToJacDotSolver::JntToJacDot,
    //                                 py::arg("q_in"), py::arg("jdot"), py::arg("seg_nr")=-1);
    // chain_jnt_to_jac_dot_solver.def("JntToJacDot", (int (KDL::ChainJntToJacDotSolver::*)(const KDL::JntArrayVel&, KDL::Twist&, int)) &KDL::ChainJntToJacDotSolver::JntToJacDot,
    //                                 py::arg("q_in"), py::arg("jac_dot_q_dot"), py::arg("seg_nr")=-1);
    // chain_jnt_to_jac_dot_solver.def("setLockedJoints", &KDL::ChainJntToJacDotSolver::setLockedJoints,
    //                                 py::arg("locked_joints"));

    // chain_jnt_to_jac_dot_solver.def_readonly_static("E_JAC_DOT_FAILED", &KDL::ChainJntToJacDotSolver::E_JAC_DOT_FAILED);
    // chain_jnt_to_jac_dot_solver.def_readonly_static("E_JACSOLVER_FAILED", &KDL::ChainJntToJacDotSolver::E_JACSOLVER_FAILED);
    // chain_jnt_to_jac_dot_solver.def_readonly_static("E_FKSOLVERPOS_FAILED", &KDL::ChainJntToJacDotSolver::E_FKSOLVERPOS_FAILED);

    // chain_jnt_to_jac_dot_solver.def_readonly_static("HYBRID", &KDL::ChainJntToJacDotSolver::HYBRID);
    // chain_jnt_to_jac_dot_solver.def_readonly_static("BODYFIXED", &KDL::ChainJntToJacDotSolver::BODYFIXED);
    // chain_jnt_to_jac_dot_solver.def_readonly_static("INERTIAL", &KDL::ChainJntToJacDotSolver::INTERTIAL);

    // chain_jnt_to_jac_dot_solver.def("setHybridRepresentation", &KDL::ChainJntToJacDotSolver::setHybridRepresentation);
    // chain_jnt_to_jac_dot_solver.def("setBodyFixedRepresentation", &KDL::ChainJntToJacDotSolver::setBodyFixedRepresentation);
    // chain_jnt_to_jac_dot_solver.def("setInternialRepresentation", &KDL::ChainJntToJacDotSolver::setInternialRepresentation);
    // chain_jnt_to_jac_dot_solver.def("setRepresentation", &KDL::ChainJntToJacDotSolver::setRepresentation,
    //                                 py::arg("representation"));

    // ChainIdSolver
    py::class_<KDL::ChainIdSolver, KDL::SolverI> chain_id_solver(m, "ChainIdSolver");
    chain_id_solver.def("CartToJnt", &KDL::ChainIdSolver::CartToJnt);

    // ChainIdSolver_RNE
    py::class_<KDL::ChainIdSolver_RNE, KDL::ChainIdSolver> chain_id_solver_RNE(m, "ChainIdSolver_RNE");
    chain_id_solver_RNE.def(py::init<const KDL::Chain&, KDL::Vector>());
}
