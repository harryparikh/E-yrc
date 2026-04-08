# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hb_interfaces:msg/BotCmd.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BotCmd(type):
    """Metaclass of message 'BotCmd'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('hb_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'hb_interfaces.msg.BotCmd')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bot_cmd
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bot_cmd
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bot_cmd
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bot_cmd
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bot_cmd

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BotCmd(metaclass=Metaclass_BotCmd):
    """Message class 'BotCmd'."""

    __slots__ = [
        '_id',
        '_m1',
        '_m2',
        '_m3',
        '_base',
        '_elbow',
    ]

    _fields_and_field_types = {
        'id': 'int32',
        'm1': 'float',
        'm2': 'float',
        'm3': 'float',
        'base': 'float',
        'elbow': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', int())
        self.m1 = kwargs.get('m1', float())
        self.m2 = kwargs.get('m2', float())
        self.m3 = kwargs.get('m3', float())
        self.base = kwargs.get('base', float())
        self.elbow = kwargs.get('elbow', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.id != other.id:
            return False
        if self.m1 != other.m1:
            return False
        if self.m2 != other.m2:
            return False
        if self.m3 != other.m3:
            return False
        if self.base != other.base:
            return False
        if self.elbow != other.elbow:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'id' field must be an integer in [-2147483648, 2147483647]"
        self._id = value

    @builtins.property
    def m1(self):
        """Message field 'm1'."""
        return self._m1

    @m1.setter
    def m1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'm1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'm1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._m1 = value

    @builtins.property
    def m2(self):
        """Message field 'm2'."""
        return self._m2

    @m2.setter
    def m2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'm2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'm2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._m2 = value

    @builtins.property
    def m3(self):
        """Message field 'm3'."""
        return self._m3

    @m3.setter
    def m3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'm3' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'm3' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._m3 = value

    @builtins.property
    def base(self):
        """Message field 'base'."""
        return self._base

    @base.setter
    def base(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'base' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'base' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._base = value

    @builtins.property
    def elbow(self):
        """Message field 'elbow'."""
        return self._elbow

    @elbow.setter
    def elbow(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'elbow' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'elbow' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._elbow = value
