include(CMakeParseArguments)

macro(generate_amw_message_headers _target)
    set(_options)
    set(_oneValueArgs OUTPUT INPUT_DIRECTORY INSTALL_DIRECTORY)
    set(_multiValueArgs MESSAGES)
    cmake_parse_arguments(_AMW_MSG "${_options}" "${_oneValueArgs}" "${_multiValueArgs}" ${ARGN})

    if (NOT _AMW_MSG_OUTPUT)
        message(FATAL_ERROR "No OUTPUT for the generated amw message definition given.")
    endif()

    if (NOT _AMW_MSG_INPUT_DIRECTORY)
        message(FATAL_ERROR "No MESSAGE DIRECTORY given.")
    endif()

    set(AMW_MESSAGE_GENERATOR ${AMW_RUNTIME_ENVIRONMENT_ROOT}/generate/sl_generate.py)
    if (NOT _AMW_MSG_MESSAGES)
        file(GLOB_RECURSE _AMW_MSG_MESSAGES CONFIGURE_DEPENDS RELATIVE ${_AMW_MSG_INPUT_DIRECTORY} "${_AMW_MSG_INPUT_DIRECTORY}/*")
    endif()
    
    set(MESSAGE_HEAD_FILES)
    foreach(_msg ${_AMW_MSG_MESSAGES})
        add_custom_command(
            OUTPUT ${_AMW_MSG_OUTPUT}/${_msg}.sl.h
            COMMAND python3 ${AMW_MESSAGE_GENERATOR} -I=${_AMW_MSG_INPUT_DIRECTORY} --cpp_out=${_AMW_MSG_OUTPUT} ${_msg}
            DEPENDS ${_AMW_MSG_INPUT_DIRECTORY}/${_msg})
        list(APPEND MESSAGE_HEAD_FILES ${_AMW_MSG_OUTPUT}/${_msg}.sl.h)
    endforeach()
    add_custom_target(${_target} DEPENDS ${MESSAGE_HEAD_FILES})

    if (_AMW_MSG_INSTALL_DIRECTORY)
        foreach(_msg ${_AMW_MSG_MESSAGES})
            STRING(FIND ${_msg} "/" _HAS_SLASH)
            if (${_HAS_SLASH} EQUAL -1)
                set(PARENT_FOLDER)
            else()
                STRING(REGEX REPLACE "(.+)/.+" "\\1" PARENT_FOLDER ${_msg})
            endif()
            install(FILES ${_AMW_MSG_INPUT_DIRECTORY}/${_msg} DESTINATION ${_AMW_MSG_INSTALL_DIRECTORY}/${PARENT_FOLDER})
        endforeach()
    endif()
endmacro()

