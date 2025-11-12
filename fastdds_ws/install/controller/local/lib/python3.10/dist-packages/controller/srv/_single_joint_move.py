# generated from rosidl_generator_py/resource/_idl.py.em
# with input from controller:srv/SingleJointMove.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SingleJointMove_Request(type):
    """Metaclass of message 'SingleJointMove_Request'."""

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
                'controller.srv.SingleJointMove_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__single_joint_move__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__single_joint_move__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__single_joint_move__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__single_joint_move__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__single_joint_move__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SingleJointMove_Request(metaclass=Metaclass_SingleJointMove_Request):
    """Message class 'SingleJointMove_Request'."""

    __slots__ = [
        '_joint_index',
        '_target_position',
        '_velocity',
    ]

    _fields_and_field_types = {
        'joint_index': 'uint32',
        'target_position': 'double',
        'velocity': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.joint_index = kwargs.get('joint_index', int())
        self.target_position = kwargs.get('target_position', float())
        self.velocity = kwargs.get('velocity', float())

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
        if self.target_position != other.target_position:
            return False
        if self.velocity != other.velocity:
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
    def target_position(self):
        """Message field 'target_position'."""
        return self._target_position

    @target_position.setter
    def target_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'target_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._target_position = value

    @builtins.property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'velocity' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._velocity = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SingleJointMove_Response(type):
    """Metaclass of message 'SingleJointMove_Response'."""

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
                'controller.srv.SingleJointMove_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__single_joint_move__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__single_joint_move__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__single_joint_move__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__single_joint_move__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__single_joint_move__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SingleJointMove_Response(metaclass=Metaclass_SingleJointMove_Response):
    """Message class 'SingleJointMove_Response'."""

    __slots__ = [
        '_success',
        '_token',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'token': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.token = kwargs.get('token', int())

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
        if self.success != other.success:
            return False
        if self.token != other.token:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def token(self):
        """Message field 'token'."""
        return self._token

    @token.setter
    def token(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'token' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'token' field must be an unsigned integer in [0, 4294967295]"
        self._token = value


class Metaclass_SingleJointMove(type):
    """Metaclass of service 'SingleJointMove'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('controller')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'controller.srv.SingleJointMove')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__single_joint_move

            from controller.srv import _single_joint_move
            if _single_joint_move.Metaclass_SingleJointMove_Request._TYPE_SUPPORT is None:
                _single_joint_move.Metaclass_SingleJointMove_Request.__import_type_support__()
            if _single_joint_move.Metaclass_SingleJointMove_Response._TYPE_SUPPORT is None:
                _single_joint_move.Metaclass_SingleJointMove_Response.__import_type_support__()


class SingleJointMove(metaclass=Metaclass_SingleJointMove):
    from controller.srv._single_joint_move import SingleJointMove_Request as Request
    from controller.srv._single_joint_move import SingleJointMove_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
