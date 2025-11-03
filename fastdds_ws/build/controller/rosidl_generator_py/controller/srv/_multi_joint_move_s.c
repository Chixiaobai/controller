// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from controller:srv/MultiJointMove.idl
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
#include "controller/srv/detail/multi_joint_move__struct.h"
#include "controller/srv/detail/multi_joint_move__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "controller/msg/detail/joint_angle__functions.h"
// end nested array functions include
bool controller__msg__joint_angle__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * controller__msg__joint_angle__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool controller__srv__multi_joint_move__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[56];
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
    assert(strncmp("controller.srv._multi_joint_move.MultiJointMove_Request", full_classname_dest, 55) == 0);
  }
  controller__srv__MultiJointMove_Request * ros_message = _ros_message;
  {  // joint_angles
    PyObject * field = PyObject_GetAttrString(_pymsg, "joint_angles");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'joint_angles'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!controller__msg__JointAngle__Sequence__init(&(ros_message->joint_angles), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create controller__msg__JointAngle__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    controller__msg__JointAngle * dest = ros_message->joint_angles.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!controller__msg__joint_angle__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * controller__srv__multi_joint_move__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MultiJointMove_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("controller.srv._multi_joint_move");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MultiJointMove_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  controller__srv__MultiJointMove_Request * ros_message = (controller__srv__MultiJointMove_Request *)raw_ros_message;
  {  // joint_angles
    PyObject * field = NULL;
    size_t size = ros_message->joint_angles.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    controller__msg__JointAngle * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->joint_angles.data[i]);
      PyObject * pyitem = controller__msg__joint_angle__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "joint_angles", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "controller/srv/detail/multi_joint_move__struct.h"
// already included above
// #include "controller/srv/detail/multi_joint_move__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool controller__srv__multi_joint_move__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[57];
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
    assert(strncmp("controller.srv._multi_joint_move.MultiJointMove_Response", full_classname_dest, 56) == 0);
  }
  controller__srv__MultiJointMove_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // token
    PyObject * field = PyObject_GetAttrString(_pymsg, "token");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->token = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * controller__srv__multi_joint_move__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MultiJointMove_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("controller.srv._multi_joint_move");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MultiJointMove_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  controller__srv__MultiJointMove_Response * ros_message = (controller__srv__MultiJointMove_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // token
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->token);
    {
      int rc = PyObject_SetAttrString(_pymessage, "token", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
