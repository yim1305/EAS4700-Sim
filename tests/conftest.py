import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import pytest

from hopper.params import load_params

CONFIG = os.path.join(ROOT, "config.yaml")


@pytest.fixture(scope="session")
def params():
    return load_params(CONFIG)


@pytest.fixture(scope="session")
def make_params():
    return lambda overrides=None: load_params(CONFIG, overrides)
