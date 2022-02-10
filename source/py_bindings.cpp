#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "lib.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i, int j)
{
  return i + j;
}

namespace py = pybind11;

PYBIND11_MODULE(_core, m)
{
  m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------
        .. currentmodule:: scikit_build_example
        .. autosummary::
           :toctree: _generate
           add
           subtract
    )pbdoc";

  m.def("add", &add, R"pbdoc(
        Add two numbers
        Some other explanation about the add function.
    )pbdoc");

  m.def("get_rho_air",
        &get_rho_air,
        R"pbdoc(get rho air)pbdoc",
        py::arg("temp_c") = 22.0,
        py::arg("elevation_m") = 180.0);

  m.def(
      "subtract", [](int i, int j) { return i - j; }, R"pbdoc(
        Subtract two numbers
        Some other explanation about the subtract function.
    )pbdoc");
  
  py::class_<TimeTrace>(m, "TimeTrace") 
    .def(py::init<vector<double>&, vector<double>&>())
    .def_readwrite("time_s", &TimeTrace::time_s)
    .def_readwrite("speed_m__s", &TimeTrace::speed_m__s)
    .def_readwrite("dt_s", &TimeTrace::dt_s)
    .def("dist_m", &TimeTrace::dist_m);


#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}