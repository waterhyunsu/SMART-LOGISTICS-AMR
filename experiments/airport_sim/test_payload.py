from types import SimpleNamespace

import pytest
from logistics_msgs.msg import TaskStep

from verify_airport import check_payload


def item(job, kind):
    return SimpleNamespace(job_id=job, kind=kind)


def test_replan_inherits_existing_bags_and_respects_limit():
    carrying = set("abcd")
    steps = {"p": item("e", TaskStep.PICKUP)}
    steps.update({key: item(key, TaskStep.DROPOFF) for key in "abcde"})
    with pytest.raises(ValueError, match="payload"):
        check_payload(["p", *"abcde"], steps, carrying)
    check_payload(["a", "p", *"bcde"], steps, carrying)
    assert carrying == set("abcd")
