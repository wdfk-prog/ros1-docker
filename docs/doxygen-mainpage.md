# ROS1 Noetic + Docker 学习工程文档站 {#mainpage}

欢迎来到 `ros1-docker` 在线文档站。

这个站点把两类内容放到一起：

- `docs/` 中面向初学者的 ROS1 Noetic + Docker 学习文章；
- `ros_ws/src/ros1_hello/` 中由 Doxygen 自动生成的源码浏览和 API 文档。

如果你第一次接触 ROS1，建议先按 01 → 05 的顺序阅读教程；如果你已经在阅读代码，可以直接从下方的 API 入口进入文件、函数和源码页面。

## 学习文档

1. [01：搭建 ROS1 Noetic Docker 开发环境](01_搭建ROS1_Noetic_Docker开发环境.md)
2. [02：创建 catkin 工作空间、Package 与第一个 Node](02_创建catkin工作空间_Package与第一个Node.md)
3. [03：让 Node 通信：Topic、Parameter 与 roslaunch](03_让Node通信_Topic_Parameter与roslaunch.md)
4. [04：使用 VS Code Remote-SSH 与 Dev Container](04_使用VSCode_RemoteSSH与DevContainer.md)
5. [05：阅读 roscpp 源码并使用 F12 / F5 调试](05_阅读roscpp源码并使用F12_F5调试.md)

## API 与源码入口

- [文件列表](files.html)：查看 `ros1_hello` 中被 Doxygen 收录的源码和文档文件。
- [全局符号](globals.html)：查看函数、变量等全局符号。
- [类型列表](annotated.html)：当工程后续加入 class/struct 时，可以从这里查看类型文档。

Doxygen 已开启 Source Browser。进入某个 `.cpp`/`.h` 文件页面后，可以继续进入对应的源码页面，对照代码、注释和调用关系阅读。

## 代码位置

当前示例 ROS1 package 位于：

```text
ros_ws/src/ros1_hello/
```

其中：

```text
src/hello_node.cpp       Publisher 示例
src/hello_listener.cpp   Subscriber 示例
launch/hello.launch      roslaunch 示例
```

本网站由 GitHub Actions 自动生成并发布到 GitHub Pages；仓库中的 Markdown 和 C/C++ 源码仍然是唯一源文件，不需要手工维护生成后的 HTML。
