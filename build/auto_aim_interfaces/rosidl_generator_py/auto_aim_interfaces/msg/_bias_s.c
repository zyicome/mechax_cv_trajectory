// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from auto_aim_interfaces:msg/Bias.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "auto_aim_interfaces/msg/detail/bias__struct.h"
#include "auto_aim_interfaces/msg/detail/bias__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool auto_aim_interfaces__msg__bias__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[35];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("auto_aim_interfaces.msg._bias.Bias", full_classname_dest, 34) == 0);
  }
  auto_aim_interfaces__msg__Bias * ros_message = _ros_message;
  {  // needchangeyaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "needchangeyaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->needchangeyaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // is_can_hit
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_can_hit");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_can_hit = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * auto_aim_interfaces__msg__bias__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Bias */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("auto_aim_interfaces.msg._bias");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Bias");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  auto_aim_interfaces__msg__Bias * ros_message = (auto_aim_interfaces__msg__Bias *)raw_ros_message;
  {  // needchangeyaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->needchangeyaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "needchangeyaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_can_hit
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_can_hit ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_can_hit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
