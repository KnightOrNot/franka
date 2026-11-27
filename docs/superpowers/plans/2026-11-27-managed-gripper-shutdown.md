# Managed gripper shutdown implementation plan

**Goal:** Stop the managed follow chain without abruptly terminating the Franka Hand client.

**Design:** Preserve startup calibration and command behavior. A separately built managed native
client acknowledges shutdown after it stops taking commands, finishes any in-flight operation, and
releases its libfranka connection. It must not issue new open, close, homing or stop-force commands
at shutdown. A managed Python launcher watches its child and closes gRPC after the child.
Keep legacy 1–4 unchanged. Use fake driver tests and compile/link on the controller without
running the actual hand. Do not claim physical restart acceptance without an operator test.

- [x] Prove the native stop path with an inert fake hand and compile/link against the project SDK.
- [x] Prove launcher signal, bounded waits, native failure propagation and gRPC close order.
- [x] Verify session Ctrl+C and web Stop share the same owned graceful hand shutdown path.
- Build and install the dedicated path using the controller deployment procedure. Physical restart
  acceptance remains an operator test.
