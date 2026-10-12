# SPDX-License-Identifier: GPL-3.0-or-later
# ironwood-cli end-to-end tests. The test runs the built CLI binary; its path is injected at configure/generate time.
iw_add_module_test(cli LIBS iw_kernel)
add_dependencies(test_cli ironwood-cli)
target_compile_definitions(test_cli PRIVATE "IRONWOOD_CLI_PATH=\"$<TARGET_FILE:ironwood-cli>\"")
