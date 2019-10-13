#!/bin/bash

# compose version string
export BINTRAY_VERSION="$(make version)-${TRAVIS_BUILD_NUMBER}"

# compile code
make EXTRA_FLAGS=-Werror ${TARGET} || exit $?

# process template
j2 bintray-template.j2 -o bintray-conf.json

cat bintray-conf.json # DEBUG
