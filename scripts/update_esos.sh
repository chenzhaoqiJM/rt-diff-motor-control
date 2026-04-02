#!/bin/bash
set -e

SRC="$1"

if [ -z "$SRC" ]; then
  echo "用法: $0 user@ip:/path/to/esos"
  exit 1
fi

mkdir -p ~/tmp_esos
cd ~/tmp_esos

echo "📥 从 $SRC 拷贝 esos 文件..."
scp "$SRC"/* .

echo "🔁 替换系统文件..."
cp esos.itb /usr/lib/riscv64-linux-gnu/esos/esos.itb
cp rt24_os0_rcpu.elf /usr/lib/riscv64-linux-gnu/esos/rt24_os0_rcpu.elf
cp rt24_os1_rcpu.elf /usr/lib/riscv64-linux-gnu/esos/rt24_os1_rcpu.elf

echo "💾 刷写 itb 到启动分区..."
dd if=/usr/lib/riscv64-linux-gnu/esos/esos.itb of=/dev/sda seek=4096 bs=1K
sync

echo "🧱 更新 initramfs..."
update-initramfs -u

echo "✅ 完成"

