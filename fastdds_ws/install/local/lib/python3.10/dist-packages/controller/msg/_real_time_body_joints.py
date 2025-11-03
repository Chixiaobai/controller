# generated from rosidl_generator_py/resource/_idl.py.em
# with input from controller:msg/RealTimeBodyJoints.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'left_arm'
# Member 'right_arm'
# Member 'torso'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RealTimeBodyJoints(type):
    """Metaclass of message 'RealTimeBodyJoints'."""

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
                'controller.msg.RealTimeBodyJoints')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__real_time_body_joints
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__real_time_body_joints
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__real_time_body_joints
            cls._TYPE_SUPPORT = module.type_support_msg__msg__real_time_body_joints
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__real_time_body_joints

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RealTimeBodyJoints(metaclass=Metaclass_RealTimeBodyJoints):
    """Message class 'RealTimeBodyJoints'."""

    __slots__ = [
        '_id',
        '_left_arm',
        '_left_arm_valid',
        '_right_arm',
        '_right_arm_valid',
        '_torso',
        '_torso_valid',
        '_time',
    ]

    _fields_and_field_types = {
        'id': 'int32',
        'left_arm': 'double[7]',
        'left_arm_valid': 'int8',
        'right_arm': 'double[7]',
        'right_arm_valid': 'int8',
        'torso': 'double[3]',
        'torso_valid': 'int8',
        'time': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 7),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 7),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', int())
        if 'left_arm' not in kwargs:
            self.left_arm = numpy.zeros(7, dtype=numpy.float64)
        else:
            self.left_arm = numpy.array(kwargs.get('left_arm'), dtype=numpy.float64)
            assert self.left_arm.shape == (7, )
        self.left_arm_valid = kwargs.get('left_arm_valid', int())
        if 'right_arm' not in kwargs:
            self.right_arm = numpy.zeros(7, dtype=numpy.float64)
        else:
            self.right_arm = numpy.array(kwargs.get('right_arm'), dtype=numpy.float64)
            assert self.right_arm.shape == (7, )
        self.right_arm_valid = kwargs.get('right_arm_valid', int())
        if 'torso' not in kwargs:
            self.torso = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.torso = numpy.array(kwargs.get('torso'), dtype=numpy.float64)
            assert self.torso.shape == (3, )
        self.torso_valid = kwargs.get('torso_valid', int())
        self.time = kwargs.get('time', float())

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
        if any(self.left_arm != other.left_arm):
            return False
        if self.left_arm_valid != other.left_arm_valid:
            return False
        if any(self.right_arm != other.right_arm):
            return False
        if self.right_arm_valid != other.right_arm_valid:
            return False
        if any(self.torso != other.torso):
            return False
        if self.torso_valid != other.torso_valid:
            return False
        if self.time != other.time:
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
    def left_arm(self):
        """Message field 'left_arm'."""
        return self._left_arm

    @left_arm.setter
    def left_arm(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'left_arm' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 7, \
                "The 'left_arm' numpy.ndarray() must have a size of 7"
            self._left_arm = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 7 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'left_arm' field must be a set or sequence with length 7 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._left_arm = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def left_arm_valid(self):
        """Message field 'left_arm_valid'."""
        return self._left_arm_valid

    @left_arm_valid.setter
    def left_arm_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'left_arm_valid' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'left_arm_valid' field must be an integer in [-128, 127]"
        self._left_arm_valid = value

    @builtins.property
    def right_arm(self):
        """Message field 'right_arm'."""
        return self._right_arm

    @right_arm.setter
    def right_arm(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'right_arm' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 7, \
                "The 'right_arm' numpy.ndarray() must have a size of 7"
            self._right_arm = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 7 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'right_arm' field must be a set or sequence with length 7 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._right_arm = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def right_arm_valid(self):
        """Message field 'right_arm_valid'."""
        return self._right_arm_valid

    @right_arm_valid.setter
    def right_arm_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'right_arm_valid' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'right_arm_valid' field must be an integer in [-128, 127]"
        self._right_arm_valid = value

    @builtins.property
    def torso(self):
        """Message field 'torso'."""
        return self._torso

    @torso.setter
    def torso(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'torso' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'torso' numpy.ndarray() must have a size of 3"
            self._torso = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'torso' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._torso = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def torso_valid(self):
        """Message field 'torso_valid'."""
        return self._torso_valid

    @torso_valid.setter
    def torso_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'torso_valid' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'torso_valid' field must be an integer in [-128, 127]"
        self._torso_valid = value

    @builtins.property
    def time(self):
        """Message field 'time'."""
        return self._time

    @time.setter
    def time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._time = value
