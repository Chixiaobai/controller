# generated from rosidl_generator_py/resource/_idl.py.em
# with input from controller:srv/GetJointMaxVel.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetJointMaxVel_Request(type):
    """Metaclass of message 'GetJointMaxVel_Request'."""

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
                'controller.srv.GetJointMaxVel_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_joint_max_vel__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_joint_max_vel__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_joint_max_vel__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_joint_max_vel__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_joint_max_vel__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetJointMaxVel_Request(metaclass=Metaclass_GetJointMaxVel_Request):
    """Message class 'GetJointMaxVel_Request'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

import builtins  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_GetJointMaxVel_Response(type):
    """Metaclass of message 'GetJointMaxVel_Response'."""

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
                'controller.srv.GetJointMaxVel_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_joint_max_vel__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_joint_max_vel__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_joint_max_vel__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_joint_max_vel__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_joint_max_vel__response

            from controller.msg import JointParams
            if JointParams.__class__._TYPE_SUPPORT is None:
                JointParams.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetJointMaxVel_Response(metaclass=Metaclass_GetJointMaxVel_Response):
    """Message class 'GetJointMaxVel_Response'."""

    __slots__ = [
        '_joint_params',
    ]

    _fields_and_field_types = {
        'joint_params': 'sequence<controller/JointParams>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['controller', 'msg'], 'JointParams')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.joint_params = kwargs.get('joint_params', [])

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
        if self.joint_params != other.joint_params:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def joint_params(self):
        """Message field 'joint_params'."""
        return self._joint_params

    @joint_params.setter
    def joint_params(self, value):
        if __debug__:
            from controller.msg import JointParams
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
                 all(isinstance(v, JointParams) for v in value) and
                 True), \
                "The 'joint_params' field must be a set or sequence and each value of type 'JointParams'"
        self._joint_params = value


class Metaclass_GetJointMaxVel(type):
    """Metaclass of service 'GetJointMaxVel'."""

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
                'controller.srv.GetJointMaxVel')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_joint_max_vel

            from controller.srv import _get_joint_max_vel
            if _get_joint_max_vel.Metaclass_GetJointMaxVel_Request._TYPE_SUPPORT is None:
                _get_joint_max_vel.Metaclass_GetJointMaxVel_Request.__import_type_support__()
            if _get_joint_max_vel.Metaclass_GetJointMaxVel_Response._TYPE_SUPPORT is None:
                _get_joint_max_vel.Metaclass_GetJointMaxVel_Response.__import_type_support__()


class GetJointMaxVel(metaclass=Metaclass_GetJointMaxVel):
    from controller.srv._get_joint_max_vel import GetJointMaxVel_Request as Request
    from controller.srv._get_joint_max_vel import GetJointMaxVel_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
