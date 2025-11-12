# generated from rosidl_generator_py/resource/_idl.py.em
# with input from controller:srv/InverseKinematics.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'reference_joint_angles'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_InverseKinematics_Request(type):
    """Metaclass of message 'InverseKinematics_Request'."""

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
                'controller.srv.InverseKinematics_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__inverse_kinematics__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__inverse_kinematics__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__inverse_kinematics__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__inverse_kinematics__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__inverse_kinematics__request

            from controller.msg import TcpPoseParams
            if TcpPoseParams.__class__._TYPE_SUPPORT is None:
                TcpPoseParams.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class InverseKinematics_Request(metaclass=Metaclass_InverseKinematics_Request):
    """Message class 'InverseKinematics_Request'."""

    __slots__ = [
        '_pose',
        '_reference_joint_angles',
        '_if_use_whole_body',
    ]

    _fields_and_field_types = {
        'pose': 'sequence<controller/TcpPoseParams>',
        'reference_joint_angles': 'sequence<double>',
        'if_use_whole_body': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['controller', 'msg'], 'TcpPoseParams')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.pose = kwargs.get('pose', [])
        self.reference_joint_angles = array.array('d', kwargs.get('reference_joint_angles', []))
        self.if_use_whole_body = kwargs.get('if_use_whole_body', bool())

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
        if self.pose != other.pose:
            return False
        if self.reference_joint_angles != other.reference_joint_angles:
            return False
        if self.if_use_whole_body != other.if_use_whole_body:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from controller.msg import TcpPoseParams
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
                 all(isinstance(v, TcpPoseParams) for v in value) and
                 True), \
                "The 'pose' field must be a set or sequence and each value of type 'TcpPoseParams'"
        self._pose = value

    @builtins.property
    def reference_joint_angles(self):
        """Message field 'reference_joint_angles'."""
        return self._reference_joint_angles

    @reference_joint_angles.setter
    def reference_joint_angles(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'reference_joint_angles' array.array() must have the type code of 'd'"
            self._reference_joint_angles = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'reference_joint_angles' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._reference_joint_angles = array.array('d', value)

    @builtins.property
    def if_use_whole_body(self):
        """Message field 'if_use_whole_body'."""
        return self._if_use_whole_body

    @if_use_whole_body.setter
    def if_use_whole_body(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'if_use_whole_body' field must be of type 'bool'"
        self._if_use_whole_body = value


# Import statements for member types

# Member 'joint_angles'
# already imported above
# import array

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_InverseKinematics_Response(type):
    """Metaclass of message 'InverseKinematics_Response'."""

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
                'controller.srv.InverseKinematics_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__inverse_kinematics__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__inverse_kinematics__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__inverse_kinematics__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__inverse_kinematics__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__inverse_kinematics__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class InverseKinematics_Response(metaclass=Metaclass_InverseKinematics_Response):
    """Message class 'InverseKinematics_Response'."""

    __slots__ = [
        '_joint_angles',
    ]

    _fields_and_field_types = {
        'joint_angles': 'sequence<double>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.joint_angles = array.array('d', kwargs.get('joint_angles', []))

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
        if self.joint_angles != other.joint_angles:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def joint_angles(self):
        """Message field 'joint_angles'."""
        return self._joint_angles

    @joint_angles.setter
    def joint_angles(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'joint_angles' array.array() must have the type code of 'd'"
            self._joint_angles = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'joint_angles' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._joint_angles = array.array('d', value)


class Metaclass_InverseKinematics(type):
    """Metaclass of service 'InverseKinematics'."""

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
                'controller.srv.InverseKinematics')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__inverse_kinematics

            from controller.srv import _inverse_kinematics
            if _inverse_kinematics.Metaclass_InverseKinematics_Request._TYPE_SUPPORT is None:
                _inverse_kinematics.Metaclass_InverseKinematics_Request.__import_type_support__()
            if _inverse_kinematics.Metaclass_InverseKinematics_Response._TYPE_SUPPORT is None:
                _inverse_kinematics.Metaclass_InverseKinematics_Response.__import_type_support__()


class InverseKinematics(metaclass=Metaclass_InverseKinematics):
    from controller.srv._inverse_kinematics import InverseKinematics_Request as Request
    from controller.srv._inverse_kinematics import InverseKinematics_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
