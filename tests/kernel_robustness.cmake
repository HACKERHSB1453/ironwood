# SPDX-License-Identifier: GPL-3.0-or-later
# K1: replayed fuzzer scenarios behind F37 (OCCT SIGSEGV in chamfer), F38 (fuse volume loss) and F39 (fillet sliver).
# Each replay must finish without a failing scenario (exit 0). Registered only when the fuzzer is part of the build.
if(TARGET ironwood-fuzz-naming)
    foreach(_seed IN ITEMS
            124307395561121837      # F37 chamfer SIGSEGV inside OCCT
            15611207460078702108    # F38 revolve join lost 93% of the body volume
            6129939700485852869     # F38 extrude join lost 1.3%
            17804925669927686653    # F38 (did not reproduce in release; kept as a guard)
            10974448563000467285    # F38 revolve join lost 20%
            9688081150919441144)    # F39 fillet sliver face ~477 mm from the body
        add_test(NAME kernel_replay_${_seed}
                 COMMAND $<TARGET_FILE:ironwood-fuzz-naming> --replay-seed ${_seed})
    endforeach()
endif()
