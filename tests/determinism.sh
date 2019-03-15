#!/bin/sh

# Copyright (c) 2019, Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0

# Arguments are passed through to sanitycheck (or cmake), for instance:
#    $0 --tag uart --tag i2c --tag build_test --tag kernel
# So are ZEPHYR_TOOLCHAIN_VARIANT etc.

# "Is the build deterministic?" is not a valid question.

# "Can I reproduce the build of my particular project even when X, Y or
# Z has changed?"  makes more sense.

# While each test_ below tries to focus on the impact of ONE specific
# change to quickly identify reproducibility issues, just like any other
# test they will sometimes catch unrelated issues too

# Of course some of these tests could pass when compiling some project
# but fail when compiling a "more demanding"/less deterministic other.

# Like other classes of bugs, https://reproducible-builds.org/ is not a
# well defined objective, it's a quest. Like any other test suite, this
# suite is an evolving trade-off to catch as many non-determinism
# issues as possible with a reasonable amount of time and resources.

# set -x
set -e


# FIXME: need a test runner:
#
# - So testing doesn't stop on the first failure
#
# - Manage test configurations. Like... sanitycheck already does? So
#   maybe all this code should be embedded in sanitycheck itself?


main()
{
    # Early prompt can save time and frustration
    rm -ri repro_build_* || true

    test_just_repeat "$@"
    test_nochange_commit "$@"
    test_change_cflags "$@"
    test_change_builddir "$@"
}



# Exclusions. Can be considered as the exhaustive "database" of
# build non-determinisms.

# Probably the quickest way to manually "test the tests" is
# to purposely break one of these exclusions by appending something
# random to it.


   ######  Files containing absolute paths ########3

# Cmake is big on absolute paths => exclude most of its files when
# building in a different directory.

# Note it is possible to compare most of these files instead of
# excluding them with something like: diff -ru
# --ignore-matching-lines='/build\(1\|2\)' build{1,2}
GREP_CMAKE_FILES='-e \.cmake$
                  -e /CMakeFiles/
                  -e /Makefile$
                  -e ./CMakeCache.txt$
                  -e /build.ninja$
                  -e /rules.ninja$
'


# Absolute paths there too, including a couple of build*/**/generated/

GREP_generated='-e /zephyr/misc/generated/syscalls_subdirs.txt$
                -e /zephyr/kconfig/sources.txt$
'

GREP_linker_cmd_dep='-e zephyr/linker.*cmd\.dep$'

# These files contains flags like -Dfu=bar.
# ./Makefile is generated by sanitycheck
# FIXME: why does CMakeError.log report a failed compiler identification?
GREP_TOOLCHAIN_FLAGS='-e /build.ninja$
                 -e /CMakeCache.txt$
                 -e flags.make$
                 -e CMakeFiles/.*\.dir/link.txt$
                 -e ./Makefile$
                 -e /CMakeFiles/CMakeError.log$
'


    ####### Changing build logs ##########

# These log files change from run to run even when nothing changes,
# exclude them always.

# - try_compile() generates a random cmTC_12345.dir/. Unless
#   --debug-trycompile is used this gets cleaned, however the random name
#   sticks in CMakeOutput.log.
# - .ninja_log has process IDs and build durations
# - Not clear why the .ninja_deps database keeps changing, timestamps maybe?
# - sanitycheck redirects stdout/stderr to build.log, make.log, not
#   deterministic with -j
GREP_RAND_BUILD_LOGS='-e /CMakeFiles/CMakeOutput.log$
                    -e /.ninja_log$
                    -e /.ninja_deps$
                    -e /build.log$
                    -e /make.log$
                    -e /config-sanitycheck.log$
'



CHKSUM=md5sum


# The most basic test. Catches the most basic issues like timestamps and
# non-determinism in python like fixed in commits 1d3dff34f5e7,
# 2eae4e5e219b, d5b2834f58e1.

# This can also catch filesystem
# non-determinism when combined with FUSE disorderedfs.
test_just_repeat()
{
    local blddir=repro_build_repeat
    $buildonce_in "$blddir" "$@"
    compute_checksums "$blddir" |
        > repeat_chksums.txt  grep -v ${GREP_RAND_BUILD_LOGS}
    mv "$blddir" "$blddir".prev
    $buildonce_in "$blddir" "$@"
    compare_checksums repeat_chksums.txt "$blddir" "$blddir".prev
}


TODO_test_fdebug_prefix_map()
{
    :
}

# Makes sure release builds don't spill any path
test_change_builddir()
{
   CMAKE_BUILD_TYPE=Release $buildonce_in repro_build_dir1/ "$@"
   compute_checksums repro_build_dir1/ |
      > chksums_subdirs.txt grep -v \
        ${GREP_RAND_BUILD_LOGS} ${GREP_linker_cmd_dep} \
        ${GREP_generated} ${GREP_CMAKE_FILES}
    CMAKE_BUILD_TYPE=Release $buildonce_in repro_build_dir2/ "$@"
    compare_checksums chksums_subdirs.txt repro_build_dir2 repro_build_dir1
}

# Tests BUILD_VERSION's ability to override git describe and reproduce
# exactly after a test or cleanup or other "nochange" commit
test_nochange_commit()
{
    local bld="repro_build_nochange_commit"
    BUILD_VERSION=myversion $buildonce_in "$bld" "$@"
    compute_checksums "$bld" |
        > commit_chksums.txt grep -v ${GREP_RAND_BUILD_LOGS}
    echo dummy_commit >> ./dummy_file
    git add ./dummy_file && git commit -m dummy_commit ./dummy_file
    mv "$bld" "$bld".prev
    BUILD_VERSION=myversion $buildonce_in "$bld" "$@"
    git reset HEAD~
    compare_checksums  commit_chksums.txt "$bld"  "$bld".prev
}


# Makes sure it's possible to reproduce even when compiler flags have
# inconsequent differences.
# To manually "test this test":
#  1. use some invalid flag and make sure the compiler complains.
#  2. Remove -gno-record-gcc-switches
test_change_cflags()
{
    local bld="repro_build_change_cflags"

    CMAKE_C_FLAGS="-Dfu=bar -gno-record-gcc-switches"  \
                 $buildonce_in "$bld" "$@"
    compute_checksums "$bld" |
        > cflags_chksums.txt grep -v ${GREP_RAND_BUILD_LOGS} \
          ${GREP_TOOLCHAIN_FLAGS}
    mv "$bld" "$bld".prev
    CMAKE_C_FLAGS="-Dbar=fu -gno-record-gcc-switches" \
                 $buildonce_in "$bld" "$@"
    compare_checksums cflags_chksums.txt  "$bld" "$bld".prev
}


die()
{
    >&2 printf "$@"
    exit 1
}


sanitycheck_buildonce_in() {
    local bld="$1"; shift
    # sanitycheck#MakeGenerator#_get_sub_make() requires one extra level
    # of quotation
     ./scripts/sanitycheck -v --build-only --outdir "$bld" \
             --extra-args=BUILD_VERSION="'${BUILD_VERSION}'" \
             --extra-args="'CMAKE_C_FLAGS=${CMAKE_C_FLAGS}'" \
             --extra-args="'CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}'" \
               "$@"  || true # when some tests fail we still want
                             # to compare the others.

     # FIME: investigate why change_cflags() causes so much ELF change
     # with zephyr-sdk-0.9.5/sysroots/x86_64-pokysdk-linux/usr/libexec/
     #        /arc-zephyr-elf/gcc/arc-zephyr-elf/6.2.1/
     # ... and not with other archs than ARC
     rm -rf "$bld"/em_starterkit_em7d/
}

buildonce_in=sanitycheck_buildonce_in

# When sanitycheck is too coarse. This can used like this:
#  - switch line above
#  - cd tests/misc/test_build/
#  - $0 -DBOARD=qemu_xtensa ...
cmake_buildonce_in() {

    local bld="$1"; shift

    mkdir "$bld"
    # Note those should be empty for most tests
    cmake -GNinja -S . -B "$bld" \
          -DBUILD_VERSION="${BUILD_VERSION}" \
          -DCMAKE_C_FLAGS="${CMAKE_C_FLAGS}" \
          -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
          "$@"
    ninja -C "$bld"  # obj_list # kobj_types_h_target 
}


compute_checksums()
{
    (cd "$1"
     find -type f -print0 | xargs -0 ${CHKSUM}
    )
}

compare_checksums()
{
    if (cd "$2" && ${CHKSUM} --quiet -c ../"$1" ); then
        printf "PASS: %s matches expected %s from first build %s PASS\n" \
               "$2" "$1" "$3"
    else
        die '%s does not match expected %s from first build %s.\n
        To diff binaries try: PATH=$path_to_objdump:$PATH diffoscope.\n' \
            "$2" "$1" "$3"
    fi
}



main "$@"

