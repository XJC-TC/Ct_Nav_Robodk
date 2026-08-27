"""Cluster checkout discovery (no RoboDK)."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from ct_nav.cluster_paths import WINDOWS_FALLBACK, cluster_candidates, discover_cluster_dir


def test_env_cluster_wins_when_it_exists(tmp_path: Path) -> None:
    cluster = tmp_path / "from-env"
    cluster.mkdir()
    found = discover_cluster_dir(
        environ={"CT_CONFIG_CLUSTER": str(cluster)},
        home=tmp_path / "unused-home",
        repo_root=tmp_path / "unused-repo",
        system="Linux",
    )
    assert found == cluster.resolve()


def test_env_cluster_skipped_when_missing_falls_through(tmp_path: Path) -> None:
    home = tmp_path / "home"
    cluster = home / "Bitbucket" / "ct_config" / "azula1"
    cluster.mkdir(parents=True)
    found = discover_cluster_dir(
        environ={"CT_CONFIG_CLUSTER": str(tmp_path / "does-not-exist")},
        home=home,
        repo_root=tmp_path / "unused-repo",
        system="Linux",
    )
    assert found == cluster.resolve()


@pytest.mark.skipif(sys.platform == "win32", reason="Windows Path equality is case-insensitive")
def test_bitbucket_capital_b_listed_before_lowercase(tmp_path: Path) -> None:
    home = tmp_path / "home"
    cands = cluster_candidates(
        environ={},
        home=home,
        repo_root=tmp_path / "unused-repo",
        system="Linux",
    )
    capital = home / "Bitbucket" / "ct_config" / "azula1"
    lower = home / "bitbucket" / "ct_config" / "azula1"
    assert cands.index(capital) < cands.index(lower)


def test_lowercase_bitbucket_when_capital_absent(tmp_path: Path) -> None:
    home = tmp_path / "home"
    lower = home / "bitbucket" / "ct_config" / "azula1"
    lower.mkdir(parents=True)
    found = discover_cluster_dir(
        environ={},
        home=home,
        repo_root=tmp_path / "unused-repo",
        system="Linux",
    )
    assert found == lower.resolve()


def test_repo_sibling_ct_config(tmp_path: Path) -> None:
    repo = tmp_path / "Ct_Nav_Robodk"
    sibling = tmp_path / "ct_config" / "azula1"
    sibling.mkdir(parents=True)
    found = discover_cluster_dir(
        environ={},
        home=tmp_path / "empty-home",
        repo_root=repo,
        system="Linux",
    )
    assert found == sibling.resolve()


def test_windows_candidates_include_drive_fallback(tmp_path: Path) -> None:
    cands = cluster_candidates(
        environ={},
        home=tmp_path / "empty-home",
        repo_root=tmp_path / "unused-repo",
        system="Windows",
    )
    assert WINDOWS_FALLBACK in cands


def test_linux_candidates_exclude_drive_fallback(tmp_path: Path) -> None:
    cands = cluster_candidates(
        environ={},
        home=tmp_path / "empty-home",
        repo_root=tmp_path / "unused-repo",
        system="Linux",
    )
    assert WINDOWS_FALLBACK not in cands


def test_nothing_found_returns_none(tmp_path: Path) -> None:
    found = discover_cluster_dir(
        environ={},
        home=tmp_path / "empty-home",
        repo_root=tmp_path / "unused-repo",
        system="Darwin",
    )
    assert found is None
