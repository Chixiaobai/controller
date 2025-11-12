# generated from rosidl_generator_py/resource/_idl.py.em
# with input from controller:srv/SetCartesianRotationMaxVel.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetCartesianRotationMaxVel_Request(type):
    """Metaclass of message 'SetCartesianRotationMaxVel_Request'."""

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
                'controller.srv.SetCartesianRotationMaxVel_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_cartesian_rotation_max_vel__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_cartesian_rotation_max_vel__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_cartesian_rotation_max_vel__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_cartesian_rotation_max_vel__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_cartesian_rotation_max_vel__request

            from controller.msg import CartesianParams
            if CartesianParams.__class__._TYPE_SUPPORT is None:
                CartesianParams.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetCartesianRotationMaxVel_Request(metaclass=Metaclass_SetCartesianRotationMaxVel_Request):
    """Message class 'SetCartesianRotationMaxVel_Request'."""

    __slots__ = [
        '_cartesian_params',
    ]

    _fields_and_field_types = {
        'cartesian_params': 'sequence<controller/CartesianParams>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['controller', 'msg'], 'CartesianParams')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.cartesian_params = kwargs.get('cartesian_params', [])

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
        if self.cartesian_params != other.cartesian_params:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def cartesian_params(self):
        """Message field 'cartesian_params'."""
        return self._cartesian_params

    @cartesian_params.setter
    def cartesian_params(self, value):
        if __debug__:
            from controller.msg import CartesianParams
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
                 all(isinstance(v, CartesianParams) for v in value) and
                 True), \
                "The 'cartesian_params' field must be a set or sequence and each value of type 'CartesianParams'"
        self._cartesian_params = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetCartesianRotationMaxVel_Response(type):
    """Metaclass of message 'SetCartesianRotationMaxVel_Response'."""

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
                'controller.srv.SetCartesianRotationMaxVel_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_cartesian_rotation_max_vel__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_cartesian_rotation_max_vel__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_cartesian_rotation_max_vel__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_cartesian_rotation_max_vel__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_cartesian_rotation_max_vel__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetCartesianRotationMaxVel_Response(metaclass=Metaclass_SetCartesianRotationMaxVel_Response):
    """Message class 'SetCartesianRotationMaxVel_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

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


class Metaclass_SetCartesianRotationMaxVel(type):
    """Metaclass of service 'SetCartesianRotationMaxVel'."""

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
                'controller.srv.SetCartesianRotationMaxVel')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_cartesian_rotation_max_vel

            from controller.srv import _set_cartesian_rotation_max_vel
            if _set_cartesian_rotation_max_vel.Metaclass_SetCartesianRotationMaxVel_Request._TYPE_SUPPORT is None:
                _set_cartesian_rotation_max_vel.Metaclass_SetCartesianRotationMaxVel_Request.__import_type_support__()
            if _set_cartesian_rotation_max_vel.Metaclass_SetCartesianRotationMaxVel_Response._TYPE_SUPPORT is None:
                _set_cartesian_rotation_max_vel.Metaclass_SetCartesianRotationMaxVel_Response.__import_type_support__()


class SetCartesianRotationMaxVel(metaclass=Metaclass_SetCartesianRotationMaxVel):
    from controller.srv._set_cartesian_rotation_max_vel import SetCartesianRotationMaxVel_Request as Request
    from controller.srv._set_cartesian_rotation_max_vel import SetCartesianRotationMaxVel_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
