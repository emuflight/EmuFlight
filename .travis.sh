#!/bin/bash

# compose version string
export EMU_VERSION="$(make version)"
export BINTRAY_VERSION="${EMU_VERSION}-${TRAVIS_BUILD_NUMBER}"

# compile code
make EXTRA_FLAGS=-Werror ${TARGET} || exit $?

# process template
j2 bintray-template.j2 -o bintray-conf.json

cat bintray-conf.json # DEBUG
