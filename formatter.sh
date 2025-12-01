#!/bin/bash

python3 -m ruff format .
python3 -m ruff --config ruff.toml check --select I --fix .
