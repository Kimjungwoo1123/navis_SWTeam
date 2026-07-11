import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

_MODULE_DIRS = [
    "Path_Planning",
    "Explore_Map",
    os.path.join("Drive", "Steer"),
    os.path.join("Drive", "Throttle"),
    os.path.join("Drive", "OutInterface"),
]

for _sub in _MODULE_DIRS:
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)
