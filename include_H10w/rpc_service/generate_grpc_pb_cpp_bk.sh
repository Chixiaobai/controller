#!/bin/bash

# 检查当前目录下是否存在 proto 目录
if [ ! -d "proto" ]; then
    echo "当前目录下不存在 proto 目录，请确认后再试。"
    exit 1
fi

# 检查当前目录下是否存在 service 目录，如果不存在则创建
rm -fr service
if [ ! -d "service" ]; then
    mkdir "service"
fi

# 遍历 proto 目录下的所有.proto 文件
for proto_file in proto/*.proto; do
    if [ -f "$proto_file" ]; then
        # 获取文件名（不包含路径）
        file_name=$(basename "$proto_file")
        # 生成 C++ 的 gRPC 和 PB 文件
        protoc --grpc_out="service" --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` --cpp_out="service" --proto_path=proto "$proto_file"
        echo "已生成 $proto_file 对应的 C++ gRPC 和 PB 文件。"
    fi
done

echo "所有操作已完成。"