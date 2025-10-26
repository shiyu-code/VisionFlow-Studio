# VisionFlow-RFlow · 粒子图像测速（PIV）演示平台

VisionFlow-RFlow 是在 VisionFlow-Studio 基础上扩展的粒子图像测速（PIV）演示软件，面向通用多相机/高速成像应用场景（如喷流、风洞、微流控、燃烧流场等）。项目以 Qt + OpenCV + FFmpeg + VTK + CMake 为核心技术栈，支持视频导入、参数化 PIV 分析、矢量/热力图可视化与结果导出。


## 功能概览（当前与规划）
- 视频导入（OpenCV/FFmpeg）：支持 mp4/avi/mkv/mov，本地文件探测显示封装与码流信息。
- PIV 分析（规划中）：
  - 预处理：去噪、直方图均衡（CLAHE）、增强/归一化、ROI 选择。
  - 互相关：多级多通道窗口（多尺度、多次迭代），支持窗口大小、重叠率、搜索范围参数化。
  - 亚像素配准：峰值拟合（抛物线/高斯），提高速度向量精度。
  - 向量后处理：中值滤波、SNR筛选、缺失插值与网格重采样。
- 可视化（Qt/VTK）：
  - 左侧：视频帧上叠加矢量箭头、速度彩色映射、掩膜/ROI显示。
  - 右侧：VTK 渲染速度场热力图、三维伪色/流线（可选）。
- 校准与坐标：像素-毫米转换、坐标轴/标尺叠加、时间序列曲线（平均速度、流量等）。
- 结果导出：CSV（矢量场）、VTK（网格/体数据）、叠加可视化 MP4/PNG。
- 工程化：CMake + vcpkg 清单模式管理依赖，跨平台可构建。

## 架构设计
- UI 层（Qt Widgets）：`src/app/MainWindow.*` 管理菜单、参数面板、播放与分析控制，左视频/右 3D 分栏布局。
- IO 层（FFmpeg/OpenCV）：`src/modules/FFmpegUtils.*` 负责媒体探测，`cv::VideoCapture` 负责读取视频帧。
- 算法层（规划）：`src/modules/piv/` 包含预处理、互相关、亚像素拟合与后处理的可插拔管线。
- 可视化层（Qt + VTK）：矢量箭头与热力图叠加；VTK 用于高级三维可视化与交互。
- 构建层（CMake）：`CMakeLists.txt` 链接 Qt/OpenCV/VTK/FFmpeg，启用 Qt AUTOMOC/AUTOUIC。

数据流：
1) 打开文件 → FFmpeg 探测 → OpenCV 解码帧 → 送入 PIV 管线 → 结果（速度向量场）
2) 叠加到视频左侧 → 在右侧以 VTK 展示热力图/流线 → 导出 CSV/VTK/MP4

## 架构图
```
[UI/Qt] MainWindow
    ├─ [IO] FFmpeg/OpenCV: probe + decode
    ├─ [Algorithms] PIV/RDIC engines
    │    ├─ Preprocess (CLAHE, normalize, ROI)
    │    ├─ Correlation / Matching
    │    └─ Postprocess (filter, resample)
    ├─ [Visualization] Qt Overlay + VTK Heatmap/Vectors
    └─ [Build] CMake + vcpkg
Data Flow: Video → Frames → Analysis → Results → Overlay/Export
```

## 目录结构（现状与规划）
```
VisionFlow-Studio/
├─ src/
│  ├─ app/                 # UI层与主窗口
│  └─ modules/
│     ├─ FFmpegUtils.*     # 媒体探测
│     └─ piv/              # 规划：PIV管线（engine/preprocess/visualization）
└─ CMakeLists.txt, vcpkg.json, README.md
```

## 构建与运行（Windows）
- 依赖：Visual Studio 2022、CMake ≥ 3.23、vcpkg（清单模式）。
- 配置与构建：
```
cmake -S . -B build-vs -G "Visual Studio 17 2022" ^
  -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" ^
  -DVCPKG_TARGET_TRIPLET=x64-windows
cmake --build build-vs --config Release
```
- 运行：`build-vs\Release\VisionFlowStudio.exe`

## 使用方式（预期工作流）
1) 启动应用 → File → Open Video，选择流场视频（推荐 H.264 MP4，英文路径）。
2) 在侧边参数面板设置：窗口大小、重叠率、搜索范围、亚像素拟合、ROI/掩膜、校准比例（像素/毫米）。
3) 点击 “Start Analysis”，运行 PIV，左侧显示矢量叠加与彩色速度图。
4) 在右侧查看 VTK 热力图/流线（可选择颜色映射、阈值）。
5) 导出结果：CSV/VTK/MP4，并保存项目配置。

## 路线图
- v0.1：单路 PIV（基础互相关 + 矢量叠加 + CSV 导出）。
- v0.2：ROI/校准、热力图与参数面板 UI，MP4 叠加导出。
- v0.3：多路相机同步、批量处理与时间序列分析。
- v0.4：集成 DIC（RDIC 类）模块，材料表面应变/位移测量。

## 与 RDIC 的关系（规划）
- 在相同框架下新增 DIC 管线：灰度散斑匹配、位移场/应变场计算、热力图与曲线可视化、导出报告。

## 许可证
MIT（学习与演示用途）。如需商用或进行系统级集成，请根据实际需求调整模块与协议。

## 新增特性（近期完成）
- ROI 编辑模式：在菜单 Analysis → Edit ROI 或按 `Ctrl+E` 开启/关闭。
- 交互句柄：ROI 四角与四边中点显示句柄，支持拖动调整大小、边缘拉伸与内部拖动移动。
- 键盘微调：在编辑模式下使用方向键移动 ROI；按住 `Shift` 以 10px 步进；按住 `Alt` 配合方向键调整宽/高。
- Qt 与 VTK 叠加一致：左侧 Qt 图像与右侧 VTK 场景同时更新 ROI，透明度与颜色保持一致。
- PIV 设置预览：在 PIV 设置对话框新增 “Preview ROI” 按钮，可不关闭窗口即时预览 ROI 位置、颜色与透明度。

## 构建提示（Qt6/vcpkg）
- 若遇到 “Could not find Qt6Config.cmake”，请设置 `CMAKE_PREFIX_PATH` 指向 Qt 安装根目录（包含 `lib/cmake/Qt6/`）：
```
cmake -S . -B build -G Ninja ^
  -DCMAKE_PREFIX_PATH="C:/Qt/6.6.3/msvc2019_64" ^
  -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake" ^
  -DVCPKG_TARGET_TRIPLET=x64-windows
cmake --build build --config Release
```
- 若之前使用了不同生成器（如 Ninja 与 VS17 混用），请清理旧的 `build/` 或重新指定新的构建目录（例如 `build-vs/`）。
- 若链接报错 `LNK1104`（无法打开 exe），请先退出正在运行的 `VisionFlowStudio.exe` 再重新构建。

## 快捷键
- `Ctrl+E`：开启/关闭 ROI 编辑模式。
- `Ctrl+R`：显示/隐藏 ROI 叠加。
- 方向键：在编辑模式下移动 ROI；`Shift` 加速步进；`Alt` 改为调整大小。

## 使用建议
- 打开视频后，先在 PIV 设置中设定 ROI 和参数，或进入编辑模式在视频上直接拖拽选区；使用预览按钮即时查看叠加效果。
- 为获得稳定的速度向量，建议合理设置窗口大小、重叠与搜索半径，并根据图像质量选择是否启用 CLAHE。

## 故障排查
- Qt6 配置：确保 `CMAKE_PREFIX_PATH` 指向 Qt6 根目录，路径下应当包含 `lib/cmake/Qt6/`。
- 生成器不一致：同一构建目录不要混用不同生成器（Ninja/VS）。
- 运行路径：尽量使用英文路径与不含空格的工作目录以避免第三方库路径解析问题。
