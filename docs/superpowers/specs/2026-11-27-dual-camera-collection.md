# Dual-camera collection and dataset batches

Keep one task name while recording multiple complete episodes. Save and discard each episode
as before. A separate Finish dataset and convert action queues exactly the saved episodes of
the current batch. It never stops following. Conversion is asynchronous and a failed job may
be retried without losing the saved batch. Archived episodes remain importable.

In dual mode require distinct serial bindings or exactly two supported attached cameras
selected in serial order. Persist stable left/right roles in the site config for installation.
Use separate RGB-D acquisition and writers. Preview choices are display-only. Status shows
observed camera rate, dimensions and frame age and the observed robot rate. One missing or stale
stream blocks new dual recordings and invalidates an active one. Raw v1 remains readable.

The session shell starts the recorder and viewer after optional one-time OS authorization.
The browser displays the configured motion target and an unchecked on-site confirmation.
An explicitly confirmed request starts the existing control chain. Subsequent starts are
non-interactive and cannot request an OS password. Stop applies only to the owned chain and
leaves the page and cameras running. Keep the original controller scripts and the no-motion
standalone capture launcher. All CLI stage, failure, and log-path text is English.

Retain the project-local runtime and data layout. Test old and new raw/LeRobot paths with synthetic
cameras, including failures and retry. Verify on the controller without starting physical motion.
