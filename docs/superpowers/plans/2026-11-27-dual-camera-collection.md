# Dual-camera collection implementation plan

> **For agentic workers:** Use superpowers:subagent-driven-development or superpowers:executing-plans. Steps use checkbox syntax.

**Goal:** Collect two RGB-D streams, group saved episodes, and control the follow lifecycle separately.

**Architecture:** Keep the managed foreground supervisor and read-only capture worker, connected
through a private same-user Unix socket. Keep the established archive pipeline and add a dual-camera
raw version plus a persisted, retryable dataset batch above it.

**Tech Stack:** Python 3.12, browser HTML/CSS/JS, existing RealSense package, LeRobot.
**Spec:** docs/superpowers/specs/2026-11-27-dual-camera-collection.md

- [ ] Add red/green dual collector and CLI tests, independent previews, observed telemetry rate, and transactional start/save.
- [ ] Add red/green raw schema, monotonic pairing, LeRobot feature and native-depth ZIP tests.
- [ ] Add red/green private follow lifecycle and English diagnostics tests with fake controller children, no physical motion.
- [ ] Add red/green persisted batch, same-task repeat saves, discard, export/retry and web route tests.
- [ ] Run full software suite and exercise dual preview/record/discard/convert/download through a simulated controller deployment.
- [ ] Publish through GitHub and verify from the project-local launchers. Document any hardware acceptance pending.
