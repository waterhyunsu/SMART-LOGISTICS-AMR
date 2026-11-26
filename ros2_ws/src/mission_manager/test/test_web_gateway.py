"""Exercise E's JSON wire contract with real A state and the DDS optimizer fixture."""
import asyncio
from types import SimpleNamespace

from aiohttp.test_utils import TestClient, TestServer

from test_dds import endpoints, until
from mission_manager.node import NavigationObservation


def test_web_order_waits_for_run_and_two_distinct_handoffs(endpoints):
    from mission_manager.web_gateway import WebMission, make_app

    manager, _, executor, optimizer = endpoints
    sends = []
    service = SimpleNamespace(service_is_ready=lambda: True)
    nav = SimpleNamespace(dispatch_client=service, lookup_client=service,
                          dispatch=lambda goal: sends.append(goal) or True)
    bridge = WebMission(manager, nav, 'r1', {'HOME', 'S4', 'G8'})

    async def scenario():
        async with TestClient(TestServer(make_app(bridge))) as client:
            async def post(path, payload, status=200):
                response = await client.post('/v1/' + path, json=payload)
                assert response.status == status, await response.text()
                return await response.json()

            async def snapshot():
                return await (await client.get('/v1/state')).json()

            mission = {'mission_id': (await snapshot())['mission_id']}
            res = await client.post('/v1/jobs', json={'job_id': 'bag1'})
            assert res.status == 422
            order = dict(mission, job_id='bag1', pickup='S4', dropoff='G8')
            assert (await post('run', mission))['outcome'] == 'rejected'
            await post('jobs', dict(order, dropoff='MISSING'), 422)
            await post('jobs', dict(order, mission_id='old'), 409)
            assert (await snapshot())['steps'] == []
            assert await post('jobs', order) == await post('jobs', order)
            await post('jobs', dict(order, pickup='HOME'), 409)
            until(lambda: manager.plan_status == 'applied', executor, optimizer)
            bridge.advance()
            assert sends == [] and not (await snapshot())['run_authorized']
            assert manager.state.latest_request_id == 1
            service.service_is_ready = lambda: False
            assert (await post('run', mission))['outcome'] == 'rejected'
            bridge.advance()
            assert sends == []
            service.service_is_ready = lambda: True
            assert (await post('run', mission))['outcome'] == 'accepted'
            assert (await post('run', mission))['outcome'] == 'accepted'
            for index, waypoint in enumerate(('S4', 'G8', 'HOME'), 1):
                bridge.advance()
                goal = sends[-1]
                assert goal.waypoint_id == waypoint and len(sends) == index
                if goal.step_id:
                    early = dict(mission, step_id=goal.step_id, event_id=f'early-{index}')
                    assert (await post('handoffs', early))['outcome'] == 'rejected'
                assert manager.handle_navigation_observation(goal, NavigationObservation.ARRIVED)
                if goal.step_id is None:
                    break
                event = dict(mission, step_id=goal.step_id, event_id=f'load-{index}')
                state = await snapshot()
                assert state['waiting_confirmation'] and state['current_goal']['waypoint_id'] == waypoint
                assert state['steps'][index-1]['kind'] == ('PICKUP' if index == 1 else 'DROPOFF')
                bridge.advance()
                assert len(sends) == index
                assert (await post('handoffs', event))['outcome'] == 'accepted'
                assert (await post('handoffs', event))['outcome'] == 'accepted'
                assert (await post('handoffs', dict(event, step_id='different')))['outcome'] == 'conflict'
            assert (await snapshot())['mission_status'] == 'completed'
            assert (await post('run', mission))['outcome'] == 'rejected'
            assert (await post('jobs', order))['job_id'] == 'bag1'

    asyncio.run(scenario())



def test_restart_recovers_ack_and_holds_new_operator_requests(tmp_path):
    from mission_manager.web_gateway import WebMission, make_app
    from test_contract import mock_optimizer
    from test_recovery_integration import manager

    db = tmp_path / 'mission.sqlite'
    service = SimpleNamespace(service_is_ready=lambda: True)
    nav = SimpleNamespace(dispatch_client=service, lookup_client=service)
    with manager(db) as node:
        bridge = WebMission(node, nav, 'r1', {'HOME', 'S4', 'G8'})
        mid = node.state.mission_id
        bridge.register(mid, 'bag', 'S4', 'G8')
        assert node.state.apply(mock_optimizer(node.request))
        # Claim after a fresh open discards the in-flight optimizer request.
    with manager(db) as node:
        bridge = WebMission(node, nav, 'r1', {'HOME', 'S4', 'G8'})
        step = node.claim_next_goal().step_id
        assert node.arrive(step)
        assert bridge.handoff(mid, step, 'loaded')['outcome'] == 'accepted'
        next_goal = node.claim_next_goal()
        assert next_goal.waypoint_id == 'G8'
        assert node.arrive(next_goal.step_id)

    with manager(db) as node:
        bridge = WebMission(node, nav, 'r1', {'HOME', 'S4', 'G8'})

        async def retry():
            async with TestClient(TestServer(make_app(bridge))) as client:
                state = await (await client.get('/v1/state')).json()
                assert state['recovery_required'] and not state['waiting_confirmation']
                for path, data, status, field, value in [
                    ('jobs', {'job_id': 'bag', 'pickup': 'S4', 'dropoff': 'G8'},
                     200, 'outcome', 'accepted'),
                    ('handoffs', {'event_id': 'loaded', 'step_id': step}, 200, 'outcome', 'accepted'),
                    ('handoffs', {'event_id': 'unload', 'step_id': next_goal.step_id},
                     409, 'error', 'recovery_pending'),
                    ('run', {}, 409, 'error', 'recovery_pending'),
                ]:
                    res = await client.post('/v1/' + path, json=dict(data, mission_id=mid))
                    assert res.status == status and (await res.json())[field] == value
                assert node.state.completed == {step} and node.current_goal() == next_goal

        asyncio.run(retry())
