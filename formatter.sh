#!/bin/bash

python -m ruff format .
python -m ruff --config ruff.toml check --select I --fix .
