# The devcontainer

How VS Code's Dev Containers extension connects to the same container the scripts
use, and the one order-dependent trap that costs a rebuild if you hit it.

## One tool creates the container

```jsonc
// .devcontainer/devcontainer.json
{
  "name": "sorting_arm",
  "dockerComposeFile": [
    "../docker-compose.yaml",
    "../.docker/compose.dev.yaml"
  ],
  "service": "dev",
  "workspaceFolder": "/sorting_arm_ws",
  "remoteUser": "rosdev",
  "updateRemoteUserUID": false,
  "shutdownAction": "none",
  ...
}
```

| Key | Why it's set this way |
|---|---|
| `dockerComposeFile` + `service` | The container is defined by the same compose file the scripts use, so there's one description of it, not two. `compose.dev.yaml` is included by default so the VS Code Server volume persists across a rebuild |
| `remoteUser` | Without it the editor lands as root, and every file it writes on the bind mount is root-owned on the host |
| `updateRemoteUserUID: false` | The image already creates `rosdev` at 1000:1000; letting the extension rewrite it re-owns files for no reason |
| `shutdownAction: none` | Closing the editor window leaves the container running - `./scripts/stop` is what actually stops it |

> [!IMPORTANT]
> **"Attach to Running Container" is not the same command as "Reopen in Container".**
> Attach connects to whatever is already running and ignores `devcontainer.json`
> entirely: no extensions install, no settings apply, and you land as the wrong user
> in the wrong directory. It doesn't error - it just quietly isn't the environment
> `devcontainer.json` describes.
>
> Concretely: if you run `./scripts/start` first and then **Attach to Running
> Container**, you get root instead of `rosdev`, none of the configured extensions,
> and none of the settings below. **Reopen in Container** is the only command that
> reads this file. Once VS Code has created the container that way, `./scripts/enter`
> or a plain `docker exec -it sorting_arm ...` from any other terminal reaches that
> same container without issue - only creating it yourself first and then attaching
> causes the problem.

## GPU: not requested by default

`devcontainer.json` can't run `docker info` to check for an NVIDIA runtime the way
`scripts/_lib.sh` does, so it doesn't request a GPU at all by default - the container
VS Code creates renders through DRI if `/dev/dri` exists on the host, or through
`llvmpipe` if it doesn't; see `docker-compose.yaml`'s fragments in
[compose.md](./compose.md). To force one, add the fragment to the array:

```jsonc
"dockerComposeFile": [
    "../docker-compose.yaml",
    "../.docker/compose.dev.yaml",
    "../.docker/compose.gpu.yaml"      // or compose.dri.yaml for integrated graphics
]
```

## The settings trap

The `settings` block under `customizations.vscode` is written once, when the
container is first created, into a machine-level settings file inside the
`sorting_arm_vscode_server` volume. A later **Rebuild Container** doesn't rewrite it.

If a setting from `devcontainer.json` refuses to take effect after you've changed it,
that persisted file is why - delete the `sorting_arm_vscode_server` volume (or run
`docker volume rm sorting_arm_vscode_server` after `./scripts/stop`) so the settings
are seeded again on the next **Reopen in Container**.

## IntelliSense choice

```jsonc
"C_Cpp.intelliSenseEngine": "default",
"C_Cpp.default.cppStandard": "c++20",
"C_Cpp.default.includePath": [
  "${workspaceFolder}/**",
  "/opt/ros/jazzy/include/**",
  "/opt/underlay/install/include/**"
]
```

The Microsoft C/C++ extension's own engine resolves symbols from this include path,
with no build database - so writing an `#include` gets completion before CMake knows
the file exists, which matters while a header and its first caller are being written
together. The BehaviorTree underlay is on the path for the same reason the ROS tree
is: neither is what `compile_commands.json` would show for a file that hasn't been
built yet.

## Extensions

The starting set in `customizations.vscode.extensions` covers this repo's languages
and formats: C++, Python, XML (launch files), YAML (configs), URDF, CMake, and an
`ament` build-task provider. Add your own on top - nothing about the list is meant to
be final, and `devcontainer.json` is a repository file like any other, so a useful
addition is worth committing.
