%module realsense
%{
  #include <rs.h>
  #include <rs.hpp>
%}

%include "exception.i"
%include "typemaps.i"
%include "carrays.i"
%include <std_vector.i>

%apply int & OUTPUT {int & width}
%apply int & OUTPUT {int & height}
%apply int & OUTPUT {int & format}
%apply int & OUTPUT {int & framerate}

%include "numpy.i"

%init %{
import_array();
%}

%exception {
  try {
    $action
  } catch (const rs::error & e){
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (const std::runtime_error& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } 
}

%array_functions(float, floatp)

%include "rs.h"
%include "rs.hpp"

%template(get_frame_data_u8) rs::device::get_frame_data<unsigned char>;
%template(get_frame_data_u16) rs::device::get_frame_data<unsigned short>;
%template(get_frame_data_f32) rs::device::get_frame_data<float>;



