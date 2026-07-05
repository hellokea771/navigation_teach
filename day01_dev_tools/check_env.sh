#!/usr/bin/env bash

set -u

required_missing=0
warning_count=0

pass() {
  printf '[PASS] %s\n' "$1"
}

warn() {
  printf '[WARN] %s\n' "$1"
  warning_count=$((warning_count + 1))
}

fail() {
  printf '[FAIL] %s\n' "$1"
  required_missing=$((required_missing + 1))
}

check_required_command() {
  local command_name="$1"
  if command -v "$command_name" >/dev/null 2>&1; then
    pass "$command_name: $(command -v "$command_name")"
  else
    fail "未找到必需命令: $command_name"
  fi
}

check_optional_command() {
  local command_name="$1"
  if command -v "$command_name" >/dev/null 2>&1; then
    pass "$command_name: $(command -v "$command_name")"
  else
    warn "未找到可选命令: $command_name"
  fi
}

printf '%s\n' '=== Day 01 开发环境检查 ==='

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  pass "操作系统: ${PRETTY_NAME:-未知 Linux 发行版}"
else
  warn '无法读取 /etc/os-release'
fi

check_required_command bash
check_required_command git
check_required_command python3
check_optional_command rg
check_optional_command code
check_optional_command gcc
check_optional_command g++
check_optional_command make
check_optional_command cmake

git_name="$(git config --global user.name 2>/dev/null || true)"
git_email="$(git config --global user.email 2>/dev/null || true)"

if [[ -n "$git_name" ]]; then
  pass "Git 用户名已设置: $git_name"
else
  warn 'Git 用户名未设置'
fi

if [[ -n "$git_email" ]]; then
  pass "Git 邮箱已设置: $git_email"
else
  warn 'Git 邮箱未设置'
fi

if git rev-parse --show-toplevel >/dev/null 2>&1; then
  pass "当前位于 Git 仓库: $(git rev-parse --show-toplevel)"
else
  warn '当前目录不在 Git 仓库中'
fi

printf '\n=== 检查结果 ===\n'
printf '缺少必需项: %d\n' "$required_missing"
printf '警告数量: %d\n' "$warning_count"

if ((required_missing > 0)); then
  printf '%s\n' '请保存以上输出并联系教师，不要自行使用 sudo 安装。'
  exit 1
fi

printf '%s\n' '必需环境检查通过。警告项可在教师指导下处理。'
