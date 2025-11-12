// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from controller:msg/CartesianParams.idl
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
#include "controller/msg/detail/cartesian_params__struct.h"
#include "controller/msg/detail/cartesian_params__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool controller__msg__cartesian_params__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
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
    assert(strncmp("controller.msg._cartesian_params.CartesianParams", full_classname_dest, 48) == 0);
  }
  controller__msg__CartesianParams * ros_message = _ros_message;
  {  // cartesian_index
    PyObject * field = PyObject_GetAttrString(_pymsg, "cartesian_index");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->cartesian_index = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // trans_max_vel
    PyObject * field = PyObject_GetAttrString(_pymsg, "trans_max_vel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->trans_max_vel = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // trans_max_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "trans_max_acc");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->trans_max_acc = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rota_max_vel
    PyObject * field = PyObject_GetAttrString(_pymsg, "rota_max_vel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rota_max_vel = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rota_max_acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "rota_max_acc");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rota_max_acc = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * controller__msg__cartesian_params__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CartesianParams */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("controller.msg._cartesian_params");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CartesianParams");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  controller__msg__CartesianParams * ros_message = (controller__msg__CartesianParams *)raw_ros_message;
  {  // cartesian_index
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->cartesian_index);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cartesian_index", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // trans_max_vel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->trans_max_vel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "trans_max_vel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // trans_max_acc
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->trans_max_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "trans_max_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rota_max_vel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rota_max_vel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rota_max_vel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rota_max_acc
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rota_max_acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rota_max_acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
