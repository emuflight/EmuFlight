# linux.mk
#
# Goals:
#   Configure an environment that will allow Taulabs GCS and firmware to be built
#   on a Linux system. The environment will support the current versions of Qt SDK
#   and the ARM toolchain installed to either the Taulabs/tools directory, their
#   respective default installation locations, or made available on the system path.

# Prefer python3; fall back to python (Python 2 EOL 2020-01-01).
ifneq ($(shell which python3),)
PYTHON:=python3
else ifneq ($(shell which python),)
PYTHON:=python
$(warning python3 not found, falling back to python. Python 2 is EOL and may cause issues.)
else
$(error Neither python3 nor python found. Please install Python 3.)
endif

export PYTHON
