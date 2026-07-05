# 第一天：Ubuntu、Git、GitHub 与 AI

## 1. 今日目标

今天不学习机器人导航算法，而是建立后续开发需要的基本工作流。

完成课程后，你应当能够：

- 使用 Ubuntu 终端完成基本文件操作。
- 看懂命令中的路径，区分绝对路径和相对路径。
- 运行程序、保存完整报错并进行初步排查。
- 理解 Git 与 GitHub 的区别。
- 完成 `clone → branch → edit → commit → push → Pull Request`。
- 使用 AI 辅助分析问题，并验证 AI 给出的建议。

今天的最终任务是：

```text
获取教学仓库
→ 创建个人分支
→ 运行一个存在问题的程序
→ 保存报错
→ 使用 AI 辅助分析
→ 修复并验证
→ 查看 Git diff
→ 提交代码
→ 创建 Pull Request
```


## 3. 开发工具全景

机器人开发通常会同时接触以下工具：

| 工具 | 作用 |
|---|---|
| Ubuntu | 机器人软件常用的开发和运行环境 |
| Terminal/Shell | 输入命令、运行程序和查看日志 |
| VS Code | 编辑代码、搜索文件和查看 Git 修改 |
| Git | 在本地记录代码版本 |
| GitHub | 托管仓库、协作和代码审查 |
| AI | 辅助理解、排错、生成和审查 |
| ROS 2 | 机器人软件模块之间的通信框架 |
| colcon/CMake | 编译和组织 ROS 2 工程 |
| RViz | 显示地图、点云、路径和坐标系 |
| rosbag | 记录和回放 ROS 2 数据 |

需要特别区分：

```text
Git：本地版本管理工具
GitHub：托管 Git 仓库的在线协作平台
```

即使没有 GitHub，Git 仍然可以在本地使用。

## 4. Ubuntu 基础

### 4.1 打开终端

Ubuntu 中可以使用快捷键：

```text
Ctrl + Alt + T
```

终端提示符通常包含用户名、主机名和当前目录。复制教程中的命令时，不要复制提示符 `$`。

### 4.2 确认当前环境

进入第一天课程目录：

```bash
cd ~/navigation_teach/day01_dev_tools
pwd
ls
```

预期 `pwd` 末尾为：

```text
/navigation_teach/day01_dev_tools
```

运行环境检查：

```bash
./check_env.sh
```

脚本会检查 Ubuntu、Git、Python、文本搜索工具和 Git 用户信息，不会安装或删除任何软件。

### 4.3 目录和路径

常用符号：

| 写法 | 含义 |
|---|---|
| `/` | Linux 根目录 |
| `~` | 当前用户的主目录 |
| `.` | 当前目录 |
| `..` | 上一级目录 |

绝对路径从 `/` 开始：

```text
/home/user/navigation_teach/day01_dev_tools
```

相对路径以当前工作目录为起点：

```text
demo/hello_robot.py
```

同一个相对路径，在不同工作目录中可能指向不同文件。这是开发中常见的问题来源。


### 4.5 运行程序和停止进程

查看 Python 版本：

```bash
python3 --version
```

运行示例程序：

```bash
python3 demo/hello_robot.py
```

现在程序应当失败。不要立即修改，先完成三件事：

1. 找到报错类型。
2. 找到报错涉及的文件路径。
3. 从第一行到最后一行保存完整报错。

运行持续工作的程序时，通常使用：

```text
Ctrl + C
```




### 4.7 Ubuntu 中的软件安装

#### 4.7.1 常见安装方式

Ubuntu 中的软件可能来自不同渠道：

| 方式 | 常见形式 | 特点 |
|---|---|---|
| APT 软件源 | `sudo apt install ...` | 自动处理 Ubuntu 软件源中的依赖，优先使用 |
| 本地 Debian 包 | `xxx.deb` | 已经编译好的 Ubuntu 安装包 |
| 语言包管理器 | `pip`、`npm` | 安装特定语言生态中的库，比如pip就是安装python库的工具 |
| 源码编译 | `CMakeLists.txt`、`Makefile` | 灵活，但需要自行处理依赖、编译和安装 |

安装软件前先确认：

1. 是否支持当前 Ubuntu 版本。
2. 是否支持当前 CPU 架构。
3. 是否会与已有版本冲突。




#### 4.7.2 使用 APT

APT 是 Ubuntu 常用的软件包管理工具。

更新本地的软件包索引：

```bash
sudo apt update
```

这条命令更新“有哪些软件和版本可用”的信息，不会自动升级所有软件。


安装软件：

```bash
sudo apt install cmake
```

同时安装多个开发工具：

```bash
sudo apt install build-essential cmake git
```

其中：

- `build-essential` 提供 GCC、G++、Make 等基础编译工具。
- `cmake` 用于配置和生成构建系统。
- `git` 用于版本管理。

查看一个包安装了哪些文件：

```bash
dpkg -L cmake
```

卸载软件：

```bash
sudo apt remove cmake
```

`remove` 通常保留系统级配置；`purge` 会同时删除包管理器记录的配置：

```bash
sudo apt purge cmake
```

不要为了练习而卸载当前课程依赖。

需要区分：

```text
apt update：更新软件包索引
apt upgrade：升级已经安装的软件(一般不用)
apt install：安装指定软件
```

课程期间不要未经确认执行整个系统的 `apt upgrade`，因为大范围升级可能改变开发环境。

#### 4.7.3 安装本地 `.deb` 包

先查看 CPU 架构：

```bash
dpkg --print-architecture
```

常见结果是：

```text
amd64
arm64
```

安装本地包时，推荐使用 APT：

```bash
sudo apt install ./example.deb
```

命令中的 `./` 表明这是当前目录中的文件，而不是软件源中的包名。

与直接运行 `dpkg -i` 相比，APT 更容易处理依赖关系。

安装前应检查：

- 文件来源。
- Ubuntu 版本。
- CPU 架构。
- 软件签名或官方校验值。
- 是否已经安装其他版本。

#### 4.7.4 Python 包

不要默认使用：

```bash
sudo pip install ...
```

它可能覆盖 Ubuntu 或 ROS 依赖的 Python 包。

项目开发更推荐虚拟环境：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install <package-name>
```

查看当前环境安装的包：

```bash
python -m pip list
python -m pip freeze
```

退出虚拟环境：

```bash
deactivate
```

虚拟环境的作用是隔离项目依赖，减少不同项目之间的版本冲突，但是实际开发接触比较少，在使用一些独立工具的时候可能会用到。

### 4.8 从源码编译和安装

#### 4.8.1 为什么需要源码编译

以下情况可能需要源码编译：

- 软件源中没有该软件。
- 软件源版本过旧。
- 需要开启或关闭某些编译选项。
- 需要修改源码。
- 目标平台没有官方安装包。

源码安装比 APT 更灵活，但也意味着开发者需要自己管理：

- 编译器。
- 依赖库。
- 编译参数。
- 安装位置。
- 版本更新。
- 卸载方式。

当你拿到一个源码压缩包之后，
确认路径安全后再解压：

```bash
tar -xf project.tar.gz
cd project
```

#### 4.8.3 CMake 项目的标准流程

机器人项目常用 CMake。一个典型流程是：

```text
准备依赖 → 配置 → 编译 → 测试 → 安装
```

流程大概是这样子：
```bash
cd 文件根目录
mkdir build //创建build文件夹
cd build 
cmake ..
make -j$(nproc)
sudo make install
```
这样子就算安装好了


#### 4.8.5 ROS 2 工作空间

ROS 2 的 `colcon` 会调用各个包的构建系统：

```bash
colcon build --symlink-install
```

常见目录：

```text
src/      源码
build/    构建中间文件
install/  安装结果和环境脚本
log/      构建日志
```

构建后通常需要：

```bash
source install/setup.bash
```

这样当前终端才能找到新编译的 ROS 2 包。

### 4.9 环境变量

#### 4.9.1 环境变量是什么

环境变量是由“名称”和“值”组成的运行环境配置，例如：

```text
HOME=/home/user
PATH=/usr/local/bin:/usr/bin:/bin
```

程序可以读取环境变量来决定：

- 去哪里寻找可执行文件。
- 去哪里寻找动态库。
- 去哪里寻找 Python 模块。
- 使用哪个配置或工作空间。
- 当前使用哪个 ROS 2 发行版。



查看环境变量：

```bash
echo "$HOME"
echo "$PATH"
printenv HOME
env | head
```



#### 4.9.3 临时设置与永久设置

终端中执行：

```bash
export COURSE_DAY=1
```

通常只对当前终端及其子进程有效。关闭终端后，该设置消失。

如果希望每次打开 Bash 都自动设置，可以写入：

```text
~/.bashrc //用nano ~/.bashrc打开
```

例如：

```bash
export PATH="$HOME/.local/bin:$PATH"
```

修改后让当前终端重新加载：

```bash
source ~/.bashrc
```

写入 `.bashrc` 前应注意：

- 先备份原文件。
- 不重复添加同一行。
- 不覆盖整个 `PATH`。
- 一次只修改一项。
- 修改后重新打开终端验证。

错误写法：

```bash
export PATH="$HOME/my_tools"
```

它会丢失系统原来的 `PATH`，导致 `ls`、`python3` 等命令可能无法找到。

更合理的写法：

```bash
export PATH="$HOME/my_tools:$PATH"
```

#### 4.9.4 常见环境变量

| 变量 | 作用 |
|---|---|
| `HOME` | 当前用户主目录 |
| `USER` | 当前用户名 |
| `SHELL` | 当前用户默认 Shell |
| `PATH` | 查找可执行程序 |
| `LD_LIBRARY_PATH` | 额外的动态库搜索路径 |
| `PYTHONPATH` | 额外的 Python 模块搜索路径 |
| `CMAKE_PREFIX_PATH` | CMake 查找已安装项目的前缀 |
| `ROS_DISTRO` | 当前 ROS 2 发行版名称 |
| `AMENT_PREFIX_PATH` | ROS 2/ament 包的安装前缀 |

。

#### 4.9.5 `source` 的作用

直接执行脚本通常会创建一个子进程。子进程修改的环境变量不会返回当前终端。

`source` 会让脚本在当前 Shell 中执行，因此脚本设置的环境变量能保留在当前终端：

```bash
source /opt/ros/<发行版>/setup.bash
source install/setup.bash
```

第一行把系统 ROS 2 环境加入当前终端；第二行再叠加当前工作空间。
我把

这种顺序称为环境叠加：

```text
Ubuntu 基础环境
→ 系统 ROS 2
→ 当前工作空间
```

在新的终端中，如果没有重新 `source`，可能出现：

```text
ros2: command not found
Package '<name>' not found
```

所以说每次编译/打开新终端，都需要在新终端输入source install/setup.bash显然很麻烦，所以需要把工作空间永久加到环境变量里，就像这样：
```bash
source /home/li/navigation2026/install/setup.bash
```
这样子每次打开新终端就不需要source了。
## 5. Git 基础

### 5.1 Git 管理的四个位置

```text
工作区
  │ git add
  ▼
暂存区
  │ git commit
  ▼
本地仓库
  │ git push
  ▼
远程仓库
```

常用命令：

| 命令 | 作用 |
|---|---|
| `git clone` | 获取远程仓库 |
| `git status` | 查看当前状态 |
| `git diff` | 查看尚未暂存的修改 |
| `git add` | 将修改放入暂存区 |
| `git commit` | 创建本地版本记录 |
| `git log` | 查看提交历史 |
| `git switch` | 切换或创建分支 |
| `git pull` | 获取并合并远程修改 |
| `git push` | 推送本地提交 |

在日常使用时，主要是会使用vscode上的git拓展



## 6. GitHub 协作

### 6.1 GitHub 中的主要对象

- Repository：项目仓库。
- Issue：任务、问题或讨论。
- Branch：相互隔离的开发分支。
- Pull Request：申请将一个分支的修改合并到另一个分支。
- Review：对 Pull Request 进行检查和评论。

### 6.2 SSH 基本概念

SSH Key 用于证明当前电脑有权访问 GitHub 账号。

检查是否已有公钥：

```bash
ls ~/.ssh
```

私钥通常没有 `.pub` 后缀，不能发送给任何人。可以上传到 GitHub 的是公钥文件内容。

如果课堂需要创建 SSH Key，由教师统一带领操作，避免学员覆盖已有密钥。

### 6.3 标准协作流程

开始任务：

```bash
git pull
git switch -c exercise/day01-<姓名拼音>
```

修改过程中：

```bash
git status
git diff
```

完成验证后：

```bash
git add day01_dev_tools/demo/hello_robot.py
git diff --staged
git commit -m "fix: locate day01 robot config"
git push -u origin exercise/day01-<姓名拼音>
```

然后在 GitHub 页面创建 Pull Request。

Pull Request 描述至少回答：

```text
修改目标：
问题原因：
修改内容：
验证方法：
AI 是否参与：
```

## 7. AI 辅助开发

### 7.1 适合 AI 的工作

- 解释命令、术语和报错。
- 根据日志提供检查顺序。
- 解释代码的输入、输出和数据流。
- 生成小段重复性代码。
- 编写测试清单。
- 审查 `git diff`。
- 改进提交信息和说明文档。

### 7.2 推荐提问模板

```text
环境：
Ubuntu 版本、Python 版本和当前工作目录。

目标：
希望程序完成什么。

执行过程：
执行了哪些命令。

实际现象：
粘贴从第一行到最后一行的完整报错。

已经检查：
列出自己已经确认的内容。

限制：
不要删除文件，不要使用 sudo，不要重置 Git 仓库。

期望回答：
先解释原因，再给检查步骤；每一步说明预期输出。
```

不推荐：

```text
程序坏了，帮我修一下。
```

因为它缺少环境、目标、操作和报错。

### 7.3 验证 AI 建议

使用 AI 后必须：

1. 理解建议涉及哪些文件和命令。
2. 检查是否包含删除、覆盖、上传或管理员权限操作。
3. 一次只执行一个排查步骤。
4. 比较实际输出与 AI 描述。
5. 修改后运行 `git diff`。
6. 重新运行程序。
7. 从不同工作目录再次测试。
8. 不提交自己无法解释的代码。

禁止向 AI 提供：

- 密码。
- GitHub Token。
- SSH 私钥。
- 未授权源码。
- 比赛策略和其他机密信息。

## 8. 综合练习

### 8.1 问题背景

`demo/hello_robot.py` 应读取 `demo/config.yaml`，然后打印机器人信息。

运行：

```bash
cd ~/navigation_teach/day01_dev_tools
python3 demo/hello_robot.py
```

当前程序会报错，这是课程故意保留的问题。

### 8.2 任务要求

1. 保存完整报错。
2. 使用 `pwd` 确认工作目录。
3. 使用 `ls` 确认配置文件实际位置。
4. 阅读 `demo/hello_robot.py`。
5. 使用推荐模板向 AI 提问。
6. 判断 AI 建议是否安全。
7. 修改程序，使它能稳定找到配置文件。
8. 比较实际输出与 `demo/expected_output.txt`。
9. 从两个工作目录验证：

   ```bash
   cd ~/navigation_teach/day01_dev_tools
   python3 demo/hello_robot.py

   cd ~/navigation_teach/day01_dev_tools/demo
   python3 hello_robot.py
   ```

10. 两次运行都必须成功。
11. 查看修改：

   ```bash
   git status
   git diff
   ```

12. 提交：

   ```bash
   git add day01_dev_tools/demo/hello_robot.py
   git diff --staged
   git commit -m "fix: locate day01 robot config"
   git push -u origin exercise/day01-<姓名拼音>
   ```

13. 在 GitHub 创建 Pull Request。

### 8.3 验收标准

- 程序可以从两个指定目录运行。
- 输出与 `expected_output.txt` 一致。
- 只修改完成任务所必需的文件。
- Commit 信息能说明修改目的。
- Pull Request 包含问题原因和验证方法。
- 能说明 AI 提供了什么建议，以及如何验证。

## 9. 常见问题

### `cd: No such file or directory`

检查：

```bash
pwd
ls
ls ~
```

确认仓库实际保存在哪里，不要假设所有人的用户名和目录相同。

### `Permission denied`

先查看：

```bash
ls -l <文件>
```

如果是自己的 Shell 脚本缺少执行权限，可以使用：

```bash
chmod +x <脚本>
```

不要使用 `sudo` 掩盖普通文件权限问题。

### `python3: command not found`

记录完整提示并运行：

```bash
which python3
echo "$PATH"
```

安装软件前先询问教师。

### `fatal: not a git repository`

说明当前目录不在 Git 仓库中。运行：

```bash
pwd
ls -la
```

找到包含 `.git` 的仓库根目录。

### `Author identity unknown`

设置 Git 用户名和邮箱：

```bash
git config --global user.name "Your Name"
git config --global user.email "your_email@example.com"
```

### Push 被拒绝

依次检查：

```bash
git status
git branch
git remote -v
```

确认正在个人分支上，并确认远程仓库地址和权限。

## 10. 课程结束检查

你应当能够回答：

1. 绝对路径和相对路径有什么区别？
2. `apt update`、`apt install` 和 `apt upgrade` 有什么区别？
3. 为什么不建议默认使用 `sudo pip install`？
4. CMake 配置、编译、测试和安装分别做什么？
5. 为什么推荐使用独立的 `build/` 目录和用户安装前缀？
6. 环境变量如何传递给子进程？
7. 临时环境变量与写入 `.bashrc` 有什么区别？
8. `PATH`、`LD_LIBRARY_PATH` 和 `CMAKE_PREFIX_PATH` 分别解决什么问题？
9. `source` 与直接运行脚本有什么区别？
10. Git 与 GitHub 有什么区别？
11. 工作区、暂存区和 Commit 分别是什么？
12. 为什么修改前后都要运行 `git status` 和 `git diff`？
13. 一个有效的 AI 排错问题应包含哪些信息？
14. 如何证明程序已经被正确修复？
15. Pull Request 中应当写明哪些内容？

完成综合练习并提交 Pull Request 后，第一天课程结束。
