%module normals
%{
  #include <normals.hpp>
%}

%include <std_vector.i>

namespace std {
  %template(VecDouble) vector<double>;
  %template(VecVecdouble) vector< vector<double> >;
}

%include "normals.hpp"



