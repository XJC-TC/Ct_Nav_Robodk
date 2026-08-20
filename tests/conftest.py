"""Shared fixtures.

Some tests run against the real ``azula1`` checkout rather than a fixture, because the
point of this project is to consume that config as-authored: a hand-written fixture
would drift and stop catching schema surprises. Those tests skip when the checkout is
absent so the suite still passes on a machine without it.
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from ct_nav import load_cluster

DEFAULT_CLUSTER = Path(r"D:/Bitbucket/ct_config/azula1")


def _cluster_dir() -> Path:
    return Path(os.environ.get("CT_CONFIG_CLUSTER", DEFAULT_CLUSTER))


@pytest.fixture(scope="session")
def cluster_dir() -> Path:
    path = _cluster_dir()
    if not path.is_dir():
        pytest.skip(f"ct_config cluster not available at {path}")
    return path


@pytest.fixture(scope="session")
def cluster(cluster_dir: Path):
    return load_cluster(cluster_dir)
