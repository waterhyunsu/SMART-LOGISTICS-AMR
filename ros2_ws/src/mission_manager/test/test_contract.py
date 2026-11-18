import pytest
from rclpy.serialization import deserialize_message, serialize_message

from logistics_msgs.msg import TaskStep
from logistics_msgs.srv import OptimizeRoute
from mission_manager.contract import MockMission


def step(key, kind, waypoint):
    return TaskStep(step_id=key, job_id='parcel', kind=kind, waypoint_id=waypoint)


@pytest.fixture
def steps():
    pickup = step('load', TaskStep.PICKUP, 'A')
    delivery = step('unload', TaskStep.DROPOFF, 'HOME')
    delivery.prerequisite_ids = ['load']
    return [pickup, delivery]


def mock_optimizer(request):
    # Exercise generated ROS types and their wire representation without DDS.
    request = deserialize_message(serialize_message(request), OptimizeRoute.Request)
    response = OptimizeRoute.Response(
        success=True, mission_id=request.mission_id, request_id=request.request_id,
        base_version=request.base_version, cost_revision=request.cost_revision,
        method='mock', estimated_distance_m=1.0,
        ordered_step_ids=[s.step_id for s in request.pending_steps])
    return deserialize_message(serialize_message(response), OptimizeRoute.Response)


@pytest.mark.parametrize('field,value', [
    ('job_id', ''), ('step_id', 'load'), ('waypoint_id', ''),
    ('kind', 9), ('prerequisite_ids', []), ('job_id', 'other'),
])
def test_invalid_job_or_step(steps, field, value):
    setattr(steps[1], field, value)
    with pytest.raises(ValueError):
        MockMission('mission', steps)


@pytest.mark.parametrize('field,value', [
    ('base_version', 7), ('mission_id', 'old'), ('success', False),
    ('ordered_step_ids', ['unload', 'load']), ('ordered_step_ids', ['load']),
    ('ordered_step_ids', ['load', 'load']), ('cost_revision', 'old'),
    ('estimated_distance_m', float('inf')),
])
def test_invalid_candidate_preserves_plan(steps, field, value):
    mission = MockMission('mission', steps)
    result = mock_optimizer(mission.request('HOME', 'map1-table1'))
    setattr(result, field, value)
    assert not mission.apply(result)
    assert mission.active_version == 0
    assert not mission.order


def test_replan_and_handoff(steps):
    mission = MockMission('mission', steps)
    initial = mock_optimizer(mission.request('HOME', 'v1'))
    assert mission.apply(initial)
    assert not mission.apply(initial)
    assert mission.start_next() == 'load'
    request = mission.request('A', 'v1')
    assert request.current_step_id == 'load'
    candidate = mock_optimizer(request)
    assert not mission.apply(candidate)
    assert not mission.confirm('mission', 'load')
    assert mission.arrive('load')
    assert not mission.confirm('old', 'load')
    assert not mission.confirm('mission', 'unload')
    assert mission.confirm('mission', 'load')
    assert not mission.confirm('mission', 'load')
    assert mission.apply(candidate)
    assert mission.active_version == 2
    assert mission.request('A', 'v1').completed_step_ids == ['load']
    assert mission.start_next() == 'unload'
    assert not mission.confirm('mission', 'load')
    assert mission.arrive('unload')
    assert mission.confirm('mission', 'unload')
    assert mission.start_next() is None


def test_new_order_invalidates_candidate_even_at_same_version(steps):
    mission = MockMission('mission', steps)
    old_result = mock_optimizer(mission.request('HOME', 'v1'))
    pickup = step('load2', TaskStep.PICKUP, 'A')
    delivery = step('unload2', TaskStep.DROPOFF, 'B')
    pickup.job_id = delivery.job_id = 'parcel2'
    delivery.prerequisite_ids = ['load2']
    mission.add_steps([pickup, delivery])
    assert not mission.apply(old_result)
    assert mission.apply(mock_optimizer(mission.request('HOME', 'v1')))


@pytest.mark.parametrize('source', ['registration', 'request'])
def test_messages_cannot_mutate_registered_prerequisites(steps, source):
    mission = MockMission('mission', steps)
    request = mission.request('HOME', 'v1')
    delivery = steps[1] if source == 'registration' else request.pending_steps[1]
    delivery.prerequisite_ids.clear()
    result = mock_optimizer(request)
    result.ordered_step_ids = ['unload', 'load']
    assert not mission.apply(result)
    assert mission.start_next() is None
    assert not mission.confirm('mission', 'unload')
    assert not mission.completed


@pytest.mark.parametrize('starts', [('HOME', 'B'), ('HOME', 'B', 'HOME')])
def test_only_latest_start_request_can_be_applied(steps, starts):
    mission = MockMission('mission', steps)
    results = [mock_optimizer(mission.request(start, 'v1')) for start in starts]
    for old in results[:-1]:
        assert not mission.apply(old)
    assert mission.active_version == 0
    assert mission.apply(results[-1])
    assert not mission.apply(results[-1])


@pytest.mark.parametrize('prerequisite', ['load2', 'unload2'])
def test_cyclic_registration_is_atomic(steps, prerequisite):
    mission = MockMission('mission', steps)
    before = mock_optimizer(mission.request('HOME', 'v1'))
    pickup = step('load2', TaskStep.PICKUP, 'B')
    delivery = step('unload2', TaskStep.DROPOFF, 'C')
    pickup.job_id = delivery.job_id = 'parcel2'
    pickup.prerequisite_ids = [prerequisite]
    delivery.prerequisite_ids = ['load2']
    with pytest.raises(ValueError, match='cycl'):
        mission.add_steps([pickup, delivery])
    assert mission.apply(before)
    assert mission.order == ['load', 'unload']
