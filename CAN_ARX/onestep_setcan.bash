#!/bin/bash

# 检查并安装必要的软件包
check_and_install_packages() {
    local packages=("can-utils" "net-tools")
    for pkg in "${packages[@]}"; do
        if ! dpkg -s "$pkg" >/dev/null 2>&1; then
            echo "正在安装 $pkg..."
            sudo apt install -y "$pkg"
        else
            echo "$pkg 已安装"
        fi
    done
}

# 初始化检查
check_and_install_packages

RULES_FILE="./arx_can.rules"
STATE_FILE="./.can_state"
SET_SCRIPT="./set.sh"

get_usb_info() {
    udevadm info -a -n /dev/ttyACM* 2>/dev/null | grep -E 'ATTRS{idVendor}|ATTRS{idProduct}|ATTRS{serial}'
}

get_serial() {
    local serial=$(get_usb_info | grep ATTRS{serial} | cut -d'"' -f2)
    [ -z "$serial" ] && echo "错误：设备序列号读取失败" >&2 && exit 1
    echo "${serial:0:12}"  # 截取前12位
}

update_rules() {
    local serial=$1
    local channel=$2
    
    # 备份原始规则文件
    cp "$RULES_FILE" "${RULES_FILE}.bak" 2>/dev/null
    
    # 转义特殊字符
    serial=$(printf "%s" "$serial" | sed -e 's/[\&/]/\\&/g')
    
    case $channel in
        1)
            sudo sed -i "/, SYMLINK+=\"arxcan1\"/s#ATTRS{serial}==\"[^\"]*\"#ATTRS{serial}==\"$serial\"#g" "$RULES_FILE"
            ;;
        3)
            sudo sed -i "/, SYMLINK+=\"arxcan3\"/s#ATTRS{serial}==\"[^\"]*\"#ATTRS{serial}==\"$serial\"#g" "$RULES_FILE"
            ;;
    esac
    
    # 显示替换差异
    echo "替换差异对比："
    diff "${RULES_FILE}.bak" "$RULES_FILE" || echo "（更新序列号）"
}

main() {
    local serial=$(get_serial)
    echo "检测到设备序列号：$serial"

    if [ ! -f "$STATE_FILE" ]; then
        channel=1
        echo "首次插入，配置左通道（CAN1）"
        touch "$STATE_FILE"
    else
        channel=3
        echo "再次插入，配置右通道（CAN3）"
        rm -f "$STATE_FILE"
    fi

    if [ -f "$RULES_FILE" ]; then
        update_rules "$serial" "$channel"
        echo "规则更新成功：$RULES_FILE"
        echo "通道 $channel 已配置为序列号：$serial"
        sudo udevadm control --reload-rules && sudo udevadm trigger
        
        # 如果是右通道（channel=3），执行set.sh
        if [ "$channel" -eq 3 ] && [ -f "$SET_SCRIPT" ]; then
            echo "正在执行 $SET_SCRIPT..."
            chmod +x "$SET_SCRIPT"  # 确保脚本有执行权限
            ./"$SET_SCRIPT"
        fi
    else
        echo "错误：未找到规则文件 $RULES_FILE" >&2
        exit 1
    fi
}

main