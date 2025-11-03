# generated from rosidl_generator_py/resource/_idl.py.em
# with input from controller:msg/JointParams.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_JointParams(type):
    """Metaclass of message 'JointParams'."""

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
            module = import_type_support('controller')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'controller.msg.JointParams')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__joint_params
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__joint_params
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__joint_params
            cls._TYPE_SUPPORT = module.type_support_msg__msg__joint_params
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__joint_params

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class JointParams(metaclass=Metaclass_JointParams):
    """Message class 'JointParams'."""

    __slots__ = [
        '_joint_index',
        '_max_pos',
        '_min_pos',
        '_max_vel',
        '_max_acc',
    ]

    _fields_and_field_types = {
        'joint_index': 'uint32',
        'max_pos': 'double',
        'min_pos': 'double',
        'max_vel': 'double',
        'max_acc': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.joint_index = kwargs.get('joint_index', int())
        self.max_pos = kwargs.get('max_pos', float())
        self.min_pos = kwargs.get('min_pos', float())
        self.max_vel = kwargs.get('max_vel', float())
        self.max_acc = kwargs.get('max_acc', float())

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
        if self.joint_index != other.joint_index:
            return False
        if self.max_pos != other.max_pos:
            return False
        if self.min_pos != other.min_pos:
            return False
        if self.max_vel != other.max_vel:
            return False
        if self.max_acc != other.max_acc:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def joint_index(self):
        """Message field 'joint_index'."""
        return self._joint_index

    @joint_index.setter
    def joint_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'joint_index' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'joint_index' field must be an unsigned integer in [0, 4294967295]"
        self._joint_index = value

    @builtins.property
    def max_pos(self):
        """Message field 'max_pos'."""
        return self._max_pos

    @max_pos.setter
    def max_pos(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_pos' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'max_pos' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._max_pos = value

    @builtins.property
    def min_pos(self):
        """Message field 'min_pos'."""
        return self._min_pos

    @min_pos.setter
    def min_pos(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'min_pos' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'min_pos' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._min_pos = value

    @builtins.property
    def max_vel(self):
        """Message field 'max_vel'."""
        return self._max_vel

    @max_vel.setter
    def max_vel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_vel' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'max_vel' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._max_vel = value

    @builtins.property
    def max_acc(self):
        """Message field 'max_acc'."""
        return self._max_acc

    @max_acc.setter
    def max_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_acc' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'max_acc' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._max_acc = value
