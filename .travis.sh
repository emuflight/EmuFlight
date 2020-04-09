#!/bin/bash
#
# travis continuous integration build script for
# Emuflight

## compile

# install crosscompiler toolchain
make arm_sdk_install

# increase the limit in order to remove Argument list too long errors
ulimit -s 131072

# compile code to binaries
make ${GOAL} || exit $?

## bintray

# get version string from 'version.h'
export EMU_VERSION="$(make version)"

# compose string to reference the artifacts (binaries)
export PACKAGE_VERSION="${TRAVIS_BUILD_NUMBER}-${EMU_VERSION}-${TRAVIS_BRANCH}"

# process template for pushing to bintray
j2 bintray-template.j2 -o bintray-conf.json
#cat bintray-conf.json # DEBUG
