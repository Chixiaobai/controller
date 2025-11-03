# 定义框架头文件搜索目录
set(TEST_BASE_INCLUDE_DIRECTORIES
    ${CMAKE_MODULE_PATH}
)

# 定义框架库文件搜索目录
set(TEST_BASE_LIB_DIRECTORIES
    ${CMAKE_MODULE_PATH}
)

# 定义框架库文件搜索目录
set(TEST_BASE_LIBRARIES
    dl
    rt
    pthread
)

# 定义框架源码
set(TEST_BASE_SRC
    ${CMAKE_MODULE_PATH}/case_api.cpp
    ${CMAKE_MODULE_PATH}/main.cpp
    ${CMAKE_MODULE_PATH}/xmlHandler.cpp
)

# 定义框架需要的宏
add_definitions(-D_USE_MATH_DEFINES -D_DISABLE_EXTENDED_ALIGNED_STORAGE)

# 定义输出后缀
set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_RELEASE_POSTFIX "")
set(CMAKE_RELWITHDEBINFO_POSTFIX "rd")
set(CMAKE_MINSIZEREL_POSTFIX "s")

# 定义CMAKE库函数
function(generate_h1_app source_list)
    add_executable(${PROJECT_NAME} ${ARGV})
    # 链接相关依赖库
    ament_target_dependencies(${PROJECT_NAME} ${COMMON_DEPS})
    # Executable files link to FastDDS-related libraries
    target_link_libraries(${PROJECT_NAME} ${GRPC_LIB} pugixml stdc++fs GTest::GTest)
    
    set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
    set_target_properties(${PROJECT_NAME} PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
    set_target_properties(${PROJECT_NAME} PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})

endfunction()

function(install_h1_app)
    install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
endfunction()
