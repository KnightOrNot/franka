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

The session shell starts the recorder and viewer. A confirmed browser start request only queues
a local terminal confirmation to start the existing control chain. Stop is available without
confirmation, applies only to the owned chain, and leaves the page and cameras running. Do not
change the original controller scripts, enable motion remotely, or introduce a browser control API
on the standalone capture launcher. All top-level CLI stage, failure, and log-path text is English.

Retain the project-local runtime and data layout. Test old and new raw/LeRobot paths with synthetic
cameras, including failures and retry. Verify on the controller without starting physical motion.
