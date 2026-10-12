# SPDX-License-Identifier: GPL-3.0-or-later
# A6: naming fuzzer smoke run (200 scenarios, < 60 s) and oracle sensitivity tests.
iw_add_module_test(fuzz_smoke LIBS iw_fuzz_naming)
