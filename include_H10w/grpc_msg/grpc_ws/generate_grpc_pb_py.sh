#!/bin/bash

# 检查是否安装了grpcio-tools
if ! python3 -c "import grpc_tools.protoc" 2>/dev/null; then
    echo "please install grpcio-tools first!"
    exit 1
fi

# 检查是否存在 Proto 目录
if [ ! -d "proto" ]; then
    echo "当前目录下不存在 Proto 目录，请确认后再试。"
    exit 1
fi

if [ ! -d "py/humanoid_monitor" ]; then
    mkdir -p py/humanoid_monitor
fi

# 生成 Python 版本 gRPC 和 PB 文件
for proto_file in proto/humanoid_monitor/*.proto; do
    if [ -f "$proto_file" ]; then
        file_name=$(basename "$proto_file")        
        python3 -m grpc_tools.protoc \
            --proto_path=proto \
            --python_out=py \
            --grpc_python_out=py \
            "$proto_file"
            
        echo "已生成 $proto_file 对应的 Python gRPC 和 PB 文件。"
    fi
done

