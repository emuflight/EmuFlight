#!/bin/bash

FC_VER=$(make version)
REVISION=$(git rev-parse --short HEAD)
BRANCH=$(git rev-parse --abbrev-ref HEAD)
REVISION=$(git rev-parse --short HEAD)
LAST_COMMIT_DATE=$(git log -1 --date=short --format="%cd")
TARGET_FILE=obj/betaflight_${FC_VER}_${TARGET}
TRAVIS_REPO_SLUG=${TRAVIS_REPO_SLUG:=$USER/undefined}
BUILDNAME=${BUILDNAME:=travis}
TRAVIS_BUILD_NUMBER=${TRAVIS_BUILD_NUMBER:=undefined}

MAKE="make EXTRA_FLAGS=-Werror"

elif [ $TARGET ] ; then
    $MAKE $TARGET || exit $?

elif [ $GOAL ] ; then
    if [ "test" == "$GOAL" ] ; then
        $MAKE check-target-independence || exit $?
        $MAKE check-fastram-usage-correctness || exit $?
    fi

    $MAKE $GOAL || exit $?

    if [ $PUBLISHCOV ] ; then
        if [ "test" == "$GOAL" ] ; then
            lcov --directory . -b src/test --capture --output-file coverage.info 2>&1 | grep -E ":version '402\*', prefer.*'406\*" --invert-match
            lcov --remove coverage.info 'lib/test/*' 'src/test/*' '/usr/*' --output-file coverage.info # filter out system and test code
            lcov --list coverage.info # debug before upload
            coveralls-lcov coverage.info # uploads to coveralls
        fi
    fi
else
    $MAKE all
fi
