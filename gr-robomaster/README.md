# gr-robomaster

`gr-robomaster` 是一个面向 RoboMaster 2-GFSK 信号接收的 GNU Radio
Out-of-Tree（OOT）模块，提供以下两个自定义块：

- `robomaster.gfsk_receiver`：目标数据帧接收器。
- `robomaster.gfsk_receiver_noise`：干扰数据帧接收器。

模块使用 C++ 实现核心处理逻辑，通过 pybind11 暴露 Python 接口，并安装
GNU Radio Companion（GRC）块描述文件。

## 目录结构

```text
gr-robomaster/
├── CMakeLists.txt
├── cmake/                       # 安装、卸载和 CMake 包配置
├── include/gnuradio/robomaster # 对外公开的 C++ 接口
├── lib/                         # C++ 私有实现
├── python/robomaster            # Python 模块和 pybind11 绑定
├── grc/                         # GNU Radio Companion 块描述
└── docs/doxygen                 # Python 绑定生成所需的最小工具集
```

`lib/` 是 GNU Radio OOT 模块的标准实现目录。公开接口放在 `include/`，具体
算法放在 `lib/`，编译后生成 `libgnuradio-robomaster.so`。

## 构建要求

- GNU Radio 3.10.x，包含开发文件；其他主版本尚未验证。
- CMake 3.16 或更高版本。
- 支持 C++17 的编译器。
- Python 3、NumPy、pybind11。
- Boost 和 GNU Radio Runtime。

## 拉取源码

```bash
git clone https://github.com/liuhansen1/RM_sdr.git
cd RM_sdr/gr-robomaster
```

## 构建

如果 CMake 能直接找到 GNU Radio：

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build --parallel
sudo cmake --install build
sudo ldconfig
```

默认安装前缀通常是 `/usr/local`。如果 GNU Radio 位于自定义目录，可以传入：

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="/path/to/gnuradio/prefix"
```

## 验证安装

使用与 GNU Radio 相同环境中的 Python 执行：

```bash
python3 -c "from gnuradio import robomaster; print(robomaster.gfsk_receiver, robomaster.gfsk_receiver_noise)"
```

也可以启动 GNU Radio Companion，搜索以下块：

```text
gfsk_receiver
gfsk_receiver_noise
```

如果 Python 找不到模块，先确认 `python3` 和 `gnuradio-config-info` 属于同一个
环境：

```bash
command -v python3
command -v gnuradio-config-info
gnuradio-config-info --prefix
```

## 修改 `lib/` 后重新构建

如果只修改了以下实现文件：

```text
lib/gfsk_receiver_impl.cc
lib/gfsk_receiver_impl.h
lib/gfsk_receiver_noise_impl.cc
lib/gfsk_receiver_noise_impl.h
```

不需要重新运行 CMake 配置，直接增量编译和安装：

```bash
cd RM_sdr/gr-robomaster
cmake --build build --parallel
cmake --install build
```

系统安装方式的最后一步使用：

```bash
sudo cmake --install build
sudo ldconfig
```

安装完成后，需要重新启动正在运行的 Python 流图或 GNU Radio Companion，旧
进程不会自动加载新共享库。

如果修改了 `CMakeLists.txt`、增加或删除了源文件，先重新配置再编译：

```bash
cmake -S . -B build
cmake --build build --parallel
cmake --install build
```

如果修改了 `include/gnuradio/robomaster/` 下的公开接口，还需要同步检查
`python/robomaster/bindings/` 中对应的 pybind11 绑定。接口发生变化时，GNU
Radio 可能要求安装 `pygccxml` 来自动重新生成绑定。

## 完全干净地重新构建

构建异常、切换编译器或切换 GNU Radio 环境时，使用干净构建：

```bash
rm -rf build

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build --parallel
sudo cmake --install build
sudo ldconfig
```

## 卸载

在原构建目录仍然存在的情况下：

```bash
cmake --build build --target uninstall
```

请确保当前构建目录对应需要卸载的安装环境。

## 开发与许可证

- `build/`、Python 缓存和本机环境路径不应提交到 Git。
- `docs/doxygen` 中保留的文件由 Python 绑定构建流程使用，不是生成的网页文档。
- `grc/*.block.yml` 用于在 GNU Radio Companion 中注册可视化块。
- `python/robomaster/bindings/` 用于提供 `gnuradio.robomaster` Python 接口。
- 模块中已有的版权和 GPL 声明继续有效，完整条款见 [COPYING](COPYING)。
- 完整系统的介绍、编译与运行见[根目录 README](../README.md)。
