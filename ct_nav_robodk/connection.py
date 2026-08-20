"""Attach to an already-running RoboDK, and only to that one.

Two defaults in ``robodk`` are wrong for this project and both bite hard on a large
station:

- The socket timeout is short enough that a ~600 MB station like NABOO-01 regularly
  fails to answer in time while it is loading or redrawing, surfacing as a bare
  ``TimeoutError`` out of ``Robolink()``.
- When that connection fails, ``Robolink`` *launches a new RoboDK*. Aim it at a station
  that is still loading and you silently end up with a second, empty instance, and every
  subsequent call talks to the wrong one.

``connect`` raises the timeout and disables the launch, so a not-yet-ready RoboDK
produces an error telling you to wait rather than a stray process.
"""

from __future__ import annotations

from robodk import robolink

DEFAULT_TIMEOUT_S = 60


class ConnectionError_(Exception):
    """Raised when RoboDK is not reachable."""


def connect(timeout_s: int = DEFAULT_TIMEOUT_S, allow_launch: bool = False) -> robolink.Robolink:
    """A ``Robolink`` bound to a running RoboDK, or a clear error explaining what to do."""
    # Robolink connects inside __init__ and reads TIMEOUT off the class while doing so, so
    # it has to be raised before construction rather than on the instance afterwards.
    previous = robolink.Robolink.TIMEOUT
    robolink.Robolink.TIMEOUT = max(timeout_s, previous)
    try:
        # An empty robodk_path leaves APPLICATION_DIR empty, which is Connect()'s signal
        # to give up instead of spawning a new instance.
        rdk = robolink.Robolink(robodk_path=None if allow_launch else "")
    except Exception as exc:
        raise ConnectionError_(_ADVICE.format(reason=exc)) from exc
    finally:
        robolink.Robolink.TIMEOUT = previous

    rdk.TIMEOUT = max(timeout_s, previous)

    # Connect() reports failure by returning 0 rather than raising, so the object above
    # can be unusable. One real call is the only reliable check.
    try:
        rdk.Command("Version")
    except Exception as exc:
        raise ConnectionError_(_ADVICE.format(reason=exc)) from exc

    return rdk


_ADVICE = (
    "Cannot connect to RoboDK ({reason}).\n"
    "Start RoboDK and wait for the station to finish loading, then try again. "
    "A large station can take a few minutes before it answers the API."
)


def active_station_name(rdk: robolink.Robolink) -> str:
    station = rdk.ActiveStation()
    return station.Name() if station.Valid() else ""
