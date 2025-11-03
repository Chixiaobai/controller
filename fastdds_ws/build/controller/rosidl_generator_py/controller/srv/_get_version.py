# generated from rosidl_generator_py/resource/_idl.py.em
# with input from controller:srv/GetVersion.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetVersion_Request(type):
    """Metaclass of message 'GetVersion_Request'."""

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
                'controller.srv.GetVersion_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_version__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_version__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_version__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_version__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_version__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetVersion_Request(metaclass=Metaclass_GetVersion_Request):
    """Message class 'GetVersion_Request'."""

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


class Metaclass_GetVersion_Response(type):
    """Metaclass of message 'GetVersion_Response'."""

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
                'controller.srv.GetVersion_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_version__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_version__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_version__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_version__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_version__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetVersion_Response(metaclass=Metaclass_GetVersion_Response):
    """Message class 'GetVersion_Response'."""

    __slots__ = [
        '_main',
        '_plugin_names',
        '_plugin_versions',
    ]

    _fields_and_field_types = {
        'main': 'string',
        'plugin_names': 'sequence<string>',
        'plugin_versions': 'sequence<string>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.main = kwargs.get('main', str())
        self.plugin_names = kwargs.get('plugin_names', [])
        self.plugin_versions = kwargs.get('plugin_versions', [])

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
        if self.main != other.main:
            return False
        if self.plugin_names != other.plugin_names:
            return False
        if self.plugin_versions != other.plugin_versions:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def main(self):
        """Message field 'main'."""
        return self._main

    @main.setter
    def main(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'main' field must be of type 'str'"
        self._main = value

    @builtins.property
    def plugin_names(self):
        """Message field 'plugin_names'."""
        return self._plugin_names

    @plugin_names.setter
    def plugin_names(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'plugin_names' field must be a set or sequence and each value of type 'str'"
        self._plugin_names = value

    @builtins.property
    def plugin_versions(self):
        """Message field 'plugin_versions'."""
        return self._plugin_versions

    @plugin_versions.setter
    def plugin_versions(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'plugin_versions' field must be a set or sequence and each value of type 'str'"
        self._plugin_versions = value


class Metaclass_GetVersion(type):
    """Metaclass of service 'GetVersion'."""

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
                'controller.srv.GetVersion')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_version

            from controller.srv import _get_version
            if _get_version.Metaclass_GetVersion_Request._TYPE_SUPPORT is None:
                _get_version.Metaclass_GetVersion_Request.__import_type_support__()
            if _get_version.Metaclass_GetVersion_Response._TYPE_SUPPORT is None:
                _get_version.Metaclass_GetVersion_Response.__import_type_support__()


class GetVersion(metaclass=Metaclass_GetVersion):
    from controller.srv._get_version import GetVersion_Request as Request
    from controller.srv._get_version import GetVersion_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
