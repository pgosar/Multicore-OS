.PHONY: check
check:
	@cd kernel && cargo clippy -- -D warnings

.PHONY: run
run:
	@cd kernel && cargo run

.PHONY: run-term
run-term:
	@cd kernel && cargo run mode terminal

.PHONY: gdb-term
gdb-term:
	@cd kernel && cargo run mode gdb-terminal

.PHONY: gdb-gui
gdb-gui:
	@cd kernel && cargo run mode gdb-gui

.PHONY: test
test:
	@cd kernel && cargo test

.PHONY: fmt
fmt:
	@cd kernel && cargo fmt

.PHONY: clean
clean:
	@cd kernel && cargo clean