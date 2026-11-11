######################################################################
#
# OrthoStream: compile options for all flight software (ADR-009)
#
# cFS applications are C99 and algorithm libraries are C++17, in the same
# build tree. add_compile_options() applies to every language in scope, so
# each standard is bound to its language with a generator expression.
# An explicit C standard also avoids the C23 default of GCC 15.
#
######################################################################

add_compile_options(
    # C: cFS applications and cFS itself
    "$<$<COMPILE_LANGUAGE:C>:-std=c99>"     # C99 without GNU extensions
    "$<$<COMPILE_LANGUAGE:C>:-pedantic>"    # strict ISO C warnings

    # C++: algorithm libraries
    "$<$<COMPILE_LANGUAGE:CXX>:-std=c++17>"

    # Both languages
    -Wall                                   # most questionable constructs
    -Werror                                 # warnings are errors
)
