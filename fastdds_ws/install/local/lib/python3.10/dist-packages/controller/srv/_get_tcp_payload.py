# generated from rosidl_generator_py/resource/_idl.py.em
# with input from controller:srv/GetTcpPayload.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'type'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetTcpPayload_Request(type):
    """Metaclass of message 'GetTcpPayload_Request'."""

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
                'controller.srv.GetTcpPayload_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_tcp_payload__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_tcp_payload__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_tcp_payload__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_tcp_payload__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_tcp_payload__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetTcpPayload_Request(metaclass=Metaclass_GetTcpPayload_Request):
    """Message class 'GetTcpPayload_Request'."""

    __slots__ = [
        '_type',
    ]

    _fields_and_field_types = {
        'type': 'sequence<int32>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.type = array.array('i', kwargs.get('type', []))

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
        if self.type != other.type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def type(self):  # noqa: A003
        """Message field 'type'."""
        return self._type

    @type.setter  # noqa: A003
    def type(self, value):  # noqa: A003
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'type' array.array() must have the type code of 'i'"
            self._type = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'type' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._type = array.array('i', value)


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetTcpPayload_Response(type):
    """Metaclass of message 'GetTcpPayload_Response'."""

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
                'controller.srv.GetTcpPayload_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_tcp_payload__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_tcp_payload__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_tcp_payload__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_tcp_payload__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_tcp_payload__response

            from controller.msg import TcpPayloadParams
            if TcpPayloadParams.__class__._TYPE_SUPPORT is None:
                TcpPayloadParams.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetTcpPayload_Response(metaclass=Metaclass_GetTcpPayload_Response):
    """Message class 'GetTcpPayload_Response'."""

    __slots__ = [
        '_success',
        '_tcp_payload_params',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'tcp_payload_params': 'sequence<controller/TcpPayloadParams>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['controller', 'msg'], 'TcpPayloadParams')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.tcp_payload_params = kwargs.get('tcp_payload_params', [])

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
        if self.tcp_payload_params != other.tcp_payload_params:
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
    def tcp_payload_params(self):
        """Message field 'tcp_payload_params'."""
        return self._tcp_payload_params

    @tcp_payload_params.setter
    def tcp_payload_params(self, value):
        if __debug__:
            from controller.msg import TcpPayloadParams
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
                 all(isinstance(v, TcpPayloadParams) for v in value) and
                 True), \
                "The 'tcp_payload_params' field must be a set or sequence and each value of type 'TcpPayloadParams'"
        self._tcp_payload_params = value


class Metaclass_GetTcpPayload(type):
    """Metaclass of service 'GetTcpPayload'."""

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
                'controller.srv.GetTcpPayload')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_tcp_payload

            from controller.srv import _get_tcp_payload
            if _get_tcp_payload.Metaclass_GetTcpPayload_Request._TYPE_SUPPORT is None:
                _get_tcp_payload.Metaclass_GetTcpPayload_Request.__import_type_support__()
            if _get_tcp_payload.Metaclass_GetTcpPayload_Response._TYPE_SUPPORT is None:
                _get_tcp_payload.Metaclass_GetTcpPayload_Response.__import_type_support__()


class GetTcpPayload(metaclass=Metaclass_GetTcpPayload):
    from controller.srv._get_tcp_payload import GetTcpPayload_Request as Request
    from controller.srv._get_tcp_payload import GetTcpPayload_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
