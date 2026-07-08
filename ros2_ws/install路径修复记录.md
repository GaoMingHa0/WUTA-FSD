# install(DIRECTORY) 路径丢失修复

> 日期: 2026-06-29

## 问题

3 个包的 CMakeLists 中 `install(DIRECTORY config/ ...)` 写法有误，config 文件被装到 `share/包名/` 根目录而非 `share/包名/config/` 子目录，导致 `ros2 launch` 找不到参数文件。

## 根因

```cmake
# 错误写法
install(DIRECTORY config/ launch/           # config/ 后面的斜杠 = "只要内容，不要目录"
  DESTINATION share/${PROJECT_NAME}         # 没写 /config → 文件散落到根目录
)
```

`config/ekf.yaml` → 装到 `share/包名/ekf.yaml`（丢失了中间 config/ 层级）

## 受影响的包

| 包 | 源码 | install 实际 | launch 期望 |
|------|------|------|------|
| localization_manager | config/ekf.yaml | share/.../ekf.yaml | share/.../config/ekf.yaml |
| controller | config/controller.yaml | share/.../controller.yaml | share/.../config/controller.yaml |
| ndt_localization | config/ndt_localization.yaml | share/.../ndt_localization.yaml | share/.../config/ndt_localization.yaml |

## 修复

```cmake
# 正确写法 — DESTINATION 里补上子目录名
install(DIRECTORY config/ launch/
  DESTINATION share/${PROJECT_NAME}/config   # ← 加 /config
)
# 或者不写尾部斜杠，保留目录结构:
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
```

参照已有正确实现的包：`cone_map_builder`、`lidar_detection`、`boundary_detector`、`path_generator`。

## 附：kiss_icp_wrapper 额外问题

该包 CMakeLists 原本完全缺少 `install(DIRECTORY config/ ...)`，已单独修复（补加 install 指令），需重编生效。
