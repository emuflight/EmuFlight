#!/bin/bash -x
#
GIT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
#GIT_COMMIT_DATE=$(git log -1 --date=short --format="%cd")
#GIT_REVISION=$(git rev-parse --short HEAD)
#TRAVIS_REPO_SLUG=${TRAVIS_REPO_SLUG:=$USER/undefined}

VERSION="$(make version)-${TRAVIS_BUILD_NUMBER}"

make EXTRA_FLAGS=-Werror ${TARGET} || exit $?

j2 bintray-template.j2 > bintray-conf.json

cat bintray-conf.json # DEBUG
