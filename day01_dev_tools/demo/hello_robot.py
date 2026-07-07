#!/usr/bin/env python3

from pathlib import Path


# 课程故意保留的问题：这里的配置文件路径不正确。
CONFIG_PATH = Path(__file__).parent / "config.yaml"


def load_simple_config(path: Path) -> dict[str, str]:
    if not path.is_file():
        raise FileNotFoundError(
            f"找不到配置文件: {path.resolve()}\n"
            "请检查当前工作目录、配置文件的实际位置和代码中的路径。"
        )

    config: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if ":" not in line:
            raise ValueError(f"配置文件第 {line_number} 行缺少冒号: {raw_line}")
        key, value = line.split(":", maxsplit=1)
        config[key.strip()] = value.strip()

    required_keys = {"robot_name", "environment", "course", "status"}
    missing_keys = sorted(required_keys - config.keys())
    if missing_keys:
        raise KeyError(f"配置文件缺少字段: {', '.join(missing_keys)}")

    return config


def main() -> None:
    config = load_simple_config(CONFIG_PATH)
    print(f"机器人名称: {config['robot_name']}")
    print(f"开发环境: {config['environment']}")
    print(f"课程: {config['course']}")
    print(f"状态: {config['status']}")


if __name__ == "__main__":
    main()

