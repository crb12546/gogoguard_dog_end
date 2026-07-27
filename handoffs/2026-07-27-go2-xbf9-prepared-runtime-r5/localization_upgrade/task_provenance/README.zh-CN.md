# 平台任务来源

本目录保存 R5 默认任务的不可变来源证据：

- `xbf9_horizontal_clean.go2-patrol-preparation.zip`：用户从平台导出的正式 ZIP；
- `source.csv`：平台上传时的原始 CSV；
- `alignment.json`：平台确认的整体 SE(2)；
- `annotations.json`：对象级固定物审核结果；
- `preparation.json`：路线裁剪、地标状态和 checkpoint 汇总。

原始 `xbf-2 2.pcd` 没有重复放在这里；运行所需的清理地图、tile、描述符和稳定层
已经编译到 `../maps/xbf9-horizontal-clean-r1/`，publication 中保留原始 PCD 的
SHA-256。若以后要从 ZIP 重新生成地图，必须提供 SHA-256 为
`3526e4f116586d3594c0afa45efb3fb254e4eca1bf89fa21f18896a558ee5aa2`
的同一份原始 PCD。

`scripts/verify_xbf_bundle_offline.py` 会逐字节核对这里的证据与实际部署地图、
路线和 checkpoint，避免现场拿错文件。
