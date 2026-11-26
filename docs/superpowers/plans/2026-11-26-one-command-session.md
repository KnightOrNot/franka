# One-command controller session

**Goal:** Start the verified root Franka sequence and the web collector with one local shell command.

**Architecture:** A foreground supervisor checks the known root configuration and startup
target, asks the on-site operator to confirm before hardware changes, and starts only its
own children. It injects the shared telemetry configuration. A small opt-in patch converts
follower opening fraction to the leader's closure coordinate for existing startup checks.
The legacy shell scripts are unchanged and use their original coordinate behavior by default.

- [x] Verify starting with both grippers open no longer fails because of opposite coordinates.
  Keep startup alignment and limits, and keep raw telemetry in its original encoding.
- [x] Check home target, stable serial, local control endpoints, ownership and idle capture.
  Supervisor starts the arm, hand, node and follow chain without global process cleanup.
- [ ] Test preflight and supervision with fake children; exercise remote check with no motion.
- [ ] Deliver the version and validate the live deployment without starting physical control.
