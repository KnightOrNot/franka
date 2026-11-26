# Franka Capture Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement each tested slice.

**Goal:** Capture single-arm Franka teleoperation and one D435i on the IPC, then check and export complete episodes offline.

**Architecture:** A Python 3.8 follower-side sender emits bounded, nonblocking cycle records. The Python 3.12 application owns recording and uses the pinned D435i library for both preview and native RGB-D. A loopback operator interface never sends robot commands.

**Tech Stack:** Python 3.12, standard-library Unix datagrams/HTTP, uv, D435i library, LeRobot 0.6.1 for offline export.

**Spec:** The operator-approved architecture from the November 26 conversation. Preserve controller ownership, seven joints plus gripper, original RGB/Z16 and timestamps, and incomplete capture evidence.

## Checks and tasks

- [x] `telemetry.py`, `patches/`: verify actual Unix transport, 8D mapping, loss handling, and patch compatibility against the existing loop. No new hardware reads or writes.
- [x] `camera.py`, `recording.py`, `episode.py`: verify a real writer with synthetic input, matched observations/actions, stale or lost input, save, discard and readback. Mark incomplete recordings as partial. Align using one host monotonic clock and report receive-time uncertainty.
- [x] `web.py`, `cli.py`, `export.py`: expose local controls, status and preview, then verify offline LeRobot readback with the 8D schema. Reject incomplete datasets and block external browser origins. Document hardware rollout and dry-run operation.

Review focus: stale feedback, sender restart, busy datagram socket, camera restarts and dropped frames, save/discard races, incorrectly repeated training samples, and out-of-root media references.
