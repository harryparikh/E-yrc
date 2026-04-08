// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from hb_interfaces:msg/BotCmd.idl
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
#include "hb_interfaces/msg/detail/bot_cmd__struct.h"
#include "hb_interfaces/msg/detail/bot_cmd__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool hb_interfaces__msg__bot_cmd__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[34];
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
    assert(strncmp("hb_interfaces.msg._bot_cmd.BotCmd", full_classname_dest, 33) == 0);
  }
  hb_interfaces__msg__BotCmd * ros_message = _ros_message;
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // m1
    PyObject * field = PyObject_GetAttrString(_pymsg, "m1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->m1 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // m2
    PyObject * field = PyObject_GetAttrString(_pymsg, "m2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->m2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // m3
    PyObject * field = PyObject_GetAttrString(_pymsg, "m3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->m3 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // base
    PyObject * field = PyObject_GetAttrString(_pymsg, "base");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->base = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // elbow
    PyObject * field = PyObject_GetAttrString(_pymsg, "elbow");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->elbow = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * hb_interfaces__msg__bot_cmd__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BotCmd */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("hb_interfaces.msg._bot_cmd");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BotCmd");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  hb_interfaces__msg__BotCmd * ros_message = (hb_interfaces__msg__BotCmd *)raw_ros_message;
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // m1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->m1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "m1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // m2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->m2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "m2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // m3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->m3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "m3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // base
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->base);
    {
      int rc = PyObject_SetAttrString(_pymessage, "base", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // elbow
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->elbow);
    {
      int rc = PyObject_SetAttrString(_pymessage, "elbow", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
