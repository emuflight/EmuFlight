#!/bin/bash
#
GIT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
GIT_COMMIT_DATE=$(git log -1 --date=short --format="%cd")
GIT_REVISION=$(git rev-parse --short HEAD)
TRAVIS_BUILD_NUMBER=${TRAVIS_BUILD_NUMBER:=undefined}
TRAVIS_REPO_SLUG=${TRAVIS_REPO_SLUG:=$USER/undefined}
CODE_VERSION=$(make version)
#
MAKE="make EXTRA_FLAGS=-Werror"

if [ $TARGET ] ; then
    $MAKE $TARGET || exit $?

elif [ $GOAL ] ; then
    if [ "test" == "$GOAL" ] ; then
        $MAKE check-target-independence || exit $?
        $MAKE check-fastram-usage-correctness || exit $?
    fi

    $MAKE $GOAL || exit $?
    jq --arg version "$CODE_VERSION.$TRAVIS_BUILD_NUMBER" --arg branch "$GIT_BRANCH" '.version.name = $version, .package.name = $branch' bintray-conf.json > bintray-conf.json
    cat bintray-conf.json

    if [ $PUBLISHCOV ] ; then
        if [ "test" == "$GOAL" ] ; then
            lcov --directory . -b src/test --capture --output-file coverage.info 2>&1 | grep -E ":version '402\*', prefer.*'406\*" --invert-match
            lcov --remove coverage.info 'lib/test/*' 'src/test/*' '/usr/*' --output-file coverage.info
            lcov --list coverage.info
            coveralls-lcov coverage.info
        fi
    fi
else
    $MAKE all
fi
