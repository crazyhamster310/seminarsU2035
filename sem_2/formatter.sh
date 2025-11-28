#!/bin/bash

ruff format .
ruff --config ruff.toml check --select I --fix .
