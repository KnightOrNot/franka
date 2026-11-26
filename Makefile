UV_RUN ?= uv run --locked

.PHONY: lint test verify test-export

lint:
	$(UV_RUN) ruff check src tests
	$(UV_RUN) ruff format --check src tests

test:
	$(UV_RUN) pytest -q

verify: lint test

test-export:
	FRANKA_TEST_EXPORT=1 $(UV_RUN) --extra export pytest -q
