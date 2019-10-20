#!/bin/bash
#
# travis continuous integration build script for
# Emuflight

# get version string from 'version.h'
export EMU_VERSION="$(make version)"

# compose string to reference the artifacts (binaries)
export PACKAGE_VERSION="${EMU_VERSION}-${TRAVIS_BUILD_NUMBER}"

# compile code to binaries
make EXTRA_FLAGS=-Werror ${GOAL} || exit $?

# process template for pushing to bintray
j2 bintray-template.j2 -o bintray-conf.json
cat bintray-conf.json # DEBUG
