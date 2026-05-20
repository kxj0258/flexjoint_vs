# Windows vcpkg 缓存 / 简易换源

仓库自带两个 PowerShell 脚本，用来给 `vcpkg` 配置本地资产缓存和简单的下载回退逻辑：

- `scripts/enable-vcpkg-cache.ps1`
- `scripts/vcpkg-asset-provider.ps1`

## 用途

适用于下面这类情况：

- `vcpkg install ...` 卡在下载 `cmake`、`ninja` 或其他 GitHub Releases 资源
- 同一台机器会重复拉取相同资产，希望本地复用缓存
- 你有一个能稳定访问的镜像前缀，希望优先走镜像

## 快速开始

在仓库根目录打开 PowerShell，先执行：

```powershell
. .\scripts\enable-vcpkg-cache.ps1 -VcpkgRoot "F:\work_kxj\tools\vcpkg"
```

这会为当前 PowerShell 会话设置：

- `VCPKG_ROOT`
- `VCPKG_LOCAL_ASSET_CACHE`
- `X_VCPKG_ASSET_SOURCES`

默认本地缓存目录是：

```text
.\.cache\vcpkg-assets
```

然后正常执行：

```powershell
F:\work_kxj\tools\vcpkg\vcpkg.exe install eigen3:x64-windows opencv4:x64-windows yaml-cpp:x64-windows
```

## 镜像前缀模式

如果你的镜像能按“保留原始 URL 路径”的方式提供资源，可以这样启用：

```powershell
. .\scripts\enable-vcpkg-cache.ps1 `
  -VcpkgRoot "F:\work_kxj\tools\vcpkg" `
  -MirrorPrefix "https://your-mirror.example.com"
```

脚本会优先尝试：

1. 镜像地址
2. 原始地址

例如原始地址如果是：

```text
https://github.com/Kitware/CMake/releases/download/v4.3.2/cmake-4.3.2-windows-x86_64.zip
```

镜像候选地址会变成：

```text
https://your-mirror.example.com/Kitware/CMake/releases/download/v4.3.2/cmake-4.3.2-windows-x86_64.zip
```

## 定向重写模式

如果你的镜像规则不是简单前缀拼接，可以直接把某个 URL 前缀替换掉：

```powershell
. .\scripts\enable-vcpkg-cache.ps1 `
  -VcpkgRoot "F:\work_kxj\tools\vcpkg" `
  -RewriteFrom "https://github.com/" `
  -RewriteTo "https://your-mirror.example.com/"
```

这时脚本会优先尝试重写后的地址，再回退到原始地址。

## 脚本行为

- 先查本地缓存
- 缓存命中时，直接把资产复制给 `vcpkg`
- 缓存未命中时，按“镜像/重写地址 -> 原始地址”的顺序下载
- 下载成功后，把文件按 `sha512` 写回本地缓存

## 注意事项

- 这些环境变量只对当前 PowerShell 会话生效
- 如果你已经单独设置过 `HTTP_PROXY` / `HTTPS_PROXY`，脚本不会覆盖它们
- 这是一个“本地缓存 + 简易换源”方案，不是完整的企业级镜像服务
- 如果某个文件已经手动放进 `vcpkg\downloads\`，`vcpkg` 通常会优先复用它
