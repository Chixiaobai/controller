# generated from rosidl_generator_py/resource/_idl.py.em
# with input from controller:msg/CartesianParams.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CartesianParams(type):
    """Metaclass of message 'CartesianParams'."""

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
                'controller.msg.CartesianParams')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__cartesian_params
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__cartesian_params
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__cartesian_params
            cls._TYPE_SUPPORT = module.type_support_msg__msg__cartesian_params
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__cartesian_params

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CartesianParams(metaclass=Metaclass_CartesianParams):
    """Message class 'CartesianParams'."""

    __slots__ = [
        '_cartesian_index',
        '_trans_max_vel',
        '_trans_max_acc',
        '_rota_max_vel',
        '_rota_max_acc',
    ]

    _fields_and_field_types = {
        'cartesian_index': 'uint32',
        'trans_max_vel': 'double',
        'trans_max_acc': 'double',
        'rota_max_vel': 'double',
        'rota_max_acc': 'double',
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
        self.cartesian_index = kwargs.get('cartesian_index', int())
        self.trans_max_vel = kwargs.get('trans_max_vel', float())
        self.trans_max_acc = kwargs.get('trans_max_acc', float())
        self.rota_max_vel = kwargs.get('rota_max_vel', float())
        self.rota_max_acc = kwargs.get('rota_max_acc', float())

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
        if self.cartesian_index != other.cartesian_index:
            return False
        if self.trans_max_vel != other.trans_max_vel:
            return False
        if self.trans_max_acc != other.trans_max_acc:
            return False
        if self.rota_max_vel != other.rota_max_vel:
            return False
        if self.rota_max_acc != other.rota_max_acc:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def cartesian_index(self):
        """Message field 'cartesian_index'."""
        return self._cartesian_index

    @cartesian_index.setter
    def cartesian_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cartesian_index' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'cartesian_index' field must be an unsigned integer in [0, 4294967295]"
        self._cartesian_index = value

    @builtins.property
    def trans_max_vel(self):
        """Message field 'trans_max_vel'."""
        return self._trans_max_vel

    @trans_max_vel.setter
    def trans_max_vel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'trans_max_vel' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'trans_max_vel' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._trans_max_vel = value

    @builtins.property
    def trans_max_acc(self):
        """Message field 'trans_max_acc'."""
        return self._trans_max_acc

    @trans_max_acc.setter
    def trans_max_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'trans_max_acc' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'trans_max_acc' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._trans_max_acc = value

    @builtins.property
    def rota_max_vel(self):
        """Message field 'rota_max_vel'."""
        return self._rota_max_vel

    @rota_max_vel.setter
    def rota_max_vel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rota_max_vel' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rota_max_vel' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rota_max_vel = value

    @builtins.property
    def rota_max_acc(self):
        """Message field 'rota_max_acc'."""
        return self._rota_max_acc

    @rota_max_acc.setter
    def rota_max_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rota_max_acc' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rota_max_acc' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rota_max_acc = value
