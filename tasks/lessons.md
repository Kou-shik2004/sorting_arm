# Lessons

Patterns that would have prevented a correction. One entry per pattern, not per incident.

## Container lifecycle is driven from VS Code, not the host shell

Koushik starts and stops the dev container through the Dev Containers extension. Host-side `docker compose` commands are for inspection only — `docker exec`, `docker compose config`, `docker ps`.

When a change to `docker-compose.yaml` or `.devcontainer/devcontainer.json` needs to take effect, give the VS Code command palette action, not the compose command. `shutdownAction: "none"` means closing the window leaves the container running, so reopening reattaches to the stale container and the change appears to have done nothing.

Read `.devcontainer/devcontainer.json` before advising on a restart. The answer is in that file.
