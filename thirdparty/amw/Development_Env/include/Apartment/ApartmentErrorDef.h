#pragma once

enum APARTMENT_ERROR {
    APARTMENT_NOT_INITIALIZED = -1,    // 库尚未被初始化
    CAN_NOT_FIND_APARTMENT = -2,       // 未找到指定apartment
    CAN_NOT_FIND_APARTMENT_NODE = -3,  // 未找到指定apartment node
    APARTMENT_METHOD_FAILED = -4,      // 指定方法执行失败
};
