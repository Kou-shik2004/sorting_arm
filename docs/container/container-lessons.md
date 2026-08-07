# Container and toolchain lessons

One pattern per correction or debugged mistake, all of them about the container, the build and the
editor. Paid for in the previous workspace and pre-seeded here so they are not paid for twice; the
full write-ups live in `ai/lessons.md`.

> Renamed from `docs/lessons.md` on 28 Jul 2026. **`ai/lessons.md` is the live pattern list** — the
> one read at session start, covering ROS, MoveIt, Gazebo and how we work. Two files called
> `lessons.md` was one source of truth too many, which is itself the first lesson below. Nothing here
> was lost in the rename; this file is the container half and it is stable.

## Comments and documentation

- **A rationale written in both a comment and a doc is one source of truth too many, and the comment is the copy that rots.** `Dockerfile:146` spent a week explaining a `PATH` problem that the `ENV PATH` line three blocks below had already solved. Two lines maximum in code; anything longer goes in `docs/` with a one-line pointer.
- **Length in a comment is not care, it is misplacement.** If it takes a paragraph, the paragraph belongs somewhere a reader goes looking for prose. The file being commented is not that place.
- **A comment that states behaviour must be tested like code.** `# Runs for every exec` on the entrypoint was simply false — `docker exec` bypasses `ENTRYPOINT` — and it had been read as true for as long as it sat there.

## Container and build

- **`docker exec` bypasses the `ENTRYPOINT`, and `bash -lc` does not rescue it.** A login shell is still non-interactive, so an interactive-guarded `bashrc` returns before sourcing anything. Environment that must reach every process is a Dockerfile `ENV`; to run a ROS command through `exec`, call the entrypoint explicitly: `docker exec <ctr> /entrypoint.sh <cmd>`.
- **`xargs` turns an empty package list into a silent no-op.** `apt-get install` with zero arguments succeeds. Read the list into a variable and guard it with `[ -n "$pkgs" ]` so the build fails at the layer instead of hours later at a missing command.
- **Moving a package list to the top of a file moves its cost with it.** Splitting lists into files is only safe if the cheap bottom slot survives — otherwise adding one utility re-runs every expensive layer beneath. Give each list its own `COPY`, and keep one list at the bottom.
- **`git clone -b <branch> --depth 1` is not a pin.** The branch moves, so identical Dockerfile text can produce different images. A `.repos` file with a commit SHA is the fix — but reproducibility is a trade, not a law: this project takes `version: humble` because an unreadable hash on a one-machine workspace costs more than it buys. Know which one you are choosing and why.
- **A named volume mounted over a config directory makes first-run provisioning pointless.** Anything installed into it survives rebuild and recreate, so a `postCreateCommand` that installs it re-checks work that could not have been lost. Ask what actually disappears between runs before automating a setup step.
- **`did not find expected <document start>` points at the wrong line.** YAML reports it where the indentation *breaks*, not where it went wrong — a single leading space on line 1 indents the whole root mapping, and the error surfaces forty lines later at the first key that returns to column 1.
- **Name a source underlay for its role, not its first occupant.** `/opt/bt_ros2_ws` forces a second workspace and a second `source` line in every shell file the day a second source dependency arrives. `/opt/underlay` with `--merge-install` never changes again.
- **An `ARG` whose value is always the same literal is not configuration.** `CODEX_VERSION=latest` looked like a knob and controlled nothing; the cache-bust argument was doing all the work.
- **Order a Dockerfile by (how often it changes, what it costs), never by topic.** Top never changes, bottom changes daily. When two layers change at a similar rate, the expensive one goes higher — a cheap edit must never re-run a slow download.
- **Declare each `ARG` directly above the `RUN` that uses it.** An `ARG`'s value joins the cache key of every instruction from its declaration downward, so a group of `ARG`s at the top turns each one into a whole-file cache bomb.
- **An unpinned "latest" installer never updates.** Docker sees identical instruction text and serves the cached layer forever. Either pin the version or give the layer a cache-bust `ARG` that the `RUN` visibly consumes — an `ARG` nothing references busts nothing.
- **Ubuntu base images defeat apt cache mounts** via `/etc/apt/apt.conf.d/docker-clean`, which deletes every `.deb` right after install. Remove it in the first layer. Then never `rm -rf /var/lib/apt/lists/*` — those paths are cache mounts and there is nothing in the image to clean.
- **`apt-get update` and `apt-get install` belong in one `RUN`.** A cached update feeding a later install produces "package not found" for packages that exist.
- **`.dockerignore` as an allowlist means every `COPY` needs a matching `!` line, in the same edit.** The error reads `failed to compute cache key: "<path>": not found`, which looks like a typo and is not.
- **BuildKit hashes file contents, not timestamps.** `touch` will not bust a cache.

## Persistence and users

- **Only named volumes and the bind mount survive a rebuild.** Anything written under `~` is in the throwaway writable layer. Before writing any fix to a dotfile, check the compose `volumes:` list.
- **A named volume inherits ownership from the image directory at first creation.** `mkdir -p` + `chown` it as root in the Dockerfile, before the `USER` switch — a later chown cannot fix a root-owned volume.
- **Never mount a volume over a home directory** — it hides everything the image put there. Mount narrow paths instead, or redirect the one file (as `HISTFILE` → `/commandhistory` does).
- **Check for sibling state directories.** A tool configured under `~/.tool` may keep its database in `~/.tool-data`, which the first volume does not cover.
- **Install anything a machine will invoke into `/usr/local/bin`, not `~/.local`.** Non-interactive shells never read `~/.bashrc`, so hooks silently fail to find the tool. Verify with `docker exec <ctr> bash -c 'command -v <tool>'` — no `-l`.
- **Ubuntu 24.04 already has a UID-1000 user.** `userdel -r ubuntu` before creating ours.
- **A plugin's requirements are system dependencies.** Its installer will happily fetch a toolchain into the writable layer on first use and lose it on every rebuild. Bake what a plugin *needs*; give a volume to what it *writes*.
- **Agent state is per-side on purpose; only configuration crosses the boundary.** Host sessions debug the container, container sessions do the work — one shared memory mixes the two. What the container should *have* is declared in the repo and applied by an idempotent creation-time script; what it *writes* stays on its own volume. Sharing the config directory itself does not work: it stores absolute paths, so each side rewrites them and breaks the other.
- **Configuration baked into an image cannot update an existing container.** A named volume is seeded from the image exactly once, at first creation. Anything that must converge on a container that already exists belongs in `postCreateCommand`, not in a `COPY`.
- **A CLI does not necessarily read the settings scope its own docs describe.** `.claude/settings.json` declares a marketplace that the in-session client honours and the `claude` CLI ignores, and the symptom is a "not found" error naming something plainly present in the file. Verify the tool sees a declaration before building on it.

## Editor and dev container

- **One tool owns the container's lifecycle.** VS Code creates it ("Reopen in Container"); the CLI only builds, starts, stops and execs. `docker compose up` + attach is the failure loop that looks like it needs a rebuild and never gets fixed by one.
- **"Attach to Running Container" ignores `devcontainer.json` entirely** — no extensions, no settings, wrong user, wrong folder, and no error to tell you.
- **Some dev-container settings are seeded once, at container creation**, into a machine-level file inside the `.vscode-server` volume, and are never rewritten by a rebuild. When a setting will not take effect, check that file before blaming the config.
- **One container, one attached editor window.** Two extension hosts collide on a lock file and commands silently vanish.
- **Delete what a removed tool generated**, in the same pass as the config change.

## Toolchain

- **PATH order is a correctness concern.** A build tool that probes a versioned binary name (`python3.12`) will find a user-local interpreter before the system one and fail on a missing module — while `which python3` still looks correct. Pin the interpreter in durable config, not in a home-dir dotfile.
- **Manifest-driven dependency resolution beats a hand-kept list.** Two sources of truth drift, and the failure shows up at runtime as "package not found". Where a hand list is used deliberately (as in the current Dockerfile, to pre-bake every phase), it has to be replaced by `rosdep` from `package.xml` as soon as manifests exist.
- **"Installed" is not "enabled".** `bash-completion` ships with its loader commented out in `/etc/bash.bashrc`.
- **A "desktop-full" base image does not contain every tool you assume.** Check, then add the gaps to the cheap tooling layer.
