"""Shared fixtures.

Some tests run against the real ``azula1`` checkout rather than a fixture, because the
point of this project is to consume that config as-authored: a hand-written fixture
would drift and stop catching schema surprises. Those tests skip when the checkout is
absent so the suite still passes on a machine without it.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from ct_nav import discover_cluster_dir, load_cluster


@pytest.fixture(scope="session")
def cluster_dir() -> Path:
    path = discover_cluster_dir()
    if path is None or not path.is_dir():
        pytest.skip(
            "ct_config cluster not available (set CT_CONFIG_CLUSTER or place azula1 "
            "at ~/Bitbucket/ct_config/azula1)"
        )
    return path


@pytest.fixture(scope="session")
def cluster(cluster_dir: Path):
    return load_cluster(cluster_dir)
