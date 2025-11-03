#!/bin/bash

# 检查当前目录下是否存在 Proto 目录
# 基础目录检查
PROTO_DIR="proto"
if [ ! -d "$PROTO_DIR" ]; then
    echo "错误：当前目录下不存在 $PROTO_DIR 目录" >&2
    exit 1
fi

# 自定义工具链路径配置
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
GRPC_BASE=$SCRIPT_DIR/../../../../thirdparty/grpc
export LD_LIBRARY_PATH="${GRPC_BASE}/lib:$LD_LIBRARY_PATH"

# 关键工具路径
PROTOC_BIN="${GRPC_BASE}/bin/protoc"
GRPC_PLUGIN="${GRPC_BASE}/bin/grpc_cpp_plugin"

# 验证工具链完整性
check_tool() {
    if [ ! -x "$1" ]; then
        echo "错误：未找到可执行文件 $1" >&2
        exit 1
    fi
}
check_tool "$PROTOC_BIN"
check_tool "$GRPC_PLUGIN"

# 输出目录处理
OUTPUT_DIR="grpc_pb_cpp"
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

# 生成处理
for proto_file in "$PROTO_DIR"/*.proto; do
    if [ -f "$proto_file" ]; then
        echo "正在处理: $(basename "$proto_file")"
        "$PROTOC_BIN" \
            --proto_path="$PROTO_DIR" \
            --proto_path="${GRPC_BASE}/include" \
            --cpp_out="$OUTPUT_DIR" \
            --grpc_out="$OUTPUT_DIR" \
            --plugin=protoc-gen-grpc="$GRPC_PLUGIN" \
            "$proto_file"
    fi
done

echo "成功生成所有文件到 $OUTPUT_DIR 目录"