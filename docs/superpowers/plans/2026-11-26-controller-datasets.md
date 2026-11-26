# Controller Dataset Workflow

> Implementation and verification are split by exporter, web interface, and deployment.

**Goal:** A controller can capture, convert and offer downloads on one loopback webpage.

**Architecture:** Keep the live Franka bridge and raw schema unchanged. The operator
chooses saved episodes from one task. A low-priority child creates an immutable
LeRobot v3 dataset and a self-contained ZIP with indexed native Z16. Do not record
and convert concurrently. A manually started controller service has no arm methods.

**Tech Stack:** Existing Python 3.12 CLI, D435i library, LeRobot 0.6.1, local HTTP and SSH.
**Spec:** User's requested AgileX-style controller workflow.

- [ ] Batch exporter: preserve original CLI contract, encode multiple episodes, native
  Z16 and calibration in download, verify independently with the pinned LeRobot reader.
- [ ] Job/catalog: publish only finalized output, validate episode IDs, handle errors,
  prevent recording/encoding overlap and reconcile shutdown.
- [ ] Web workflow: saved-run selection and task grouping, background job status,
  downloadable artifact, local and same-origin boundaries. Exercise HTTP flow.
- [ ] Deployment: one manual startup command, locked camera + exporter dependencies,
  public GitHub clone at verified revision, existing site env and optics settings.
- [ ] Verify, review, publish to GitHub, and check controller using existing complete raw
  and live camera preview without motion. Preserve the old deployment for rollback.
