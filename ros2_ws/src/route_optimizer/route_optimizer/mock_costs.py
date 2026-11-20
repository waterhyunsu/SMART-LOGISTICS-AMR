"""Fixed MOCK cost snapshot used for local service verification."""

from types import MappingProxyType

COST_REVISION = "research2-mock-v1"
WAYPOINTS = ("HOME", "A", "B", "C", "D")
COST_ROWS = (
    (0, 3, 6, 8, 9),
    (4, 0, 2, 5, 6),
    (6, 3, 0, 2, 4),
    (5, 4, 3, 0, 2),
    (7, 6, 3, 4, 0),
)
COST = MappingProxyType({
    (start, end): distance
    for start, row in zip(WAYPOINTS, COST_ROWS)
    for end, distance in zip(WAYPOINTS, row)
})
