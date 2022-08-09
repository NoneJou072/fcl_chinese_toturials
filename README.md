# FCL 开源碰撞检测库的简单使用

## 1. 安装
仓库地址：https://github.com/flexible-collision-library/fcl  
  
可以将其 clone 至工作空间的 src 目录下，然后回到工作空间执行  
```bash
    colcon build --packages-select fcl
```
测试执行编译好的可执行文件：
```bash
    ./build/fcl/test/test_fcl_collision
```
终端中能够正常打印出gtest测试文本即测试完成

## 2. 入门
在 code 文件夹中有一篇入门用的样例，将其拖入 `fcl/test` 目录中，然后在此目录下的 `CMakeLists.txt` 中插入：
```
# test file list
set(tests
    ...
    zhr_test_fcl_distance.cpp
)
```
保存后重新编译，重新执行上述操作，测试样例是否可正常执行。
具体的内容请查看 doc 文件夹中的文档。