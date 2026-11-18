"""Test-only OptimizeRoute server. Echo the supplied order without optimizing."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from logistics_msgs.srv import OptimizeRoute


class MockOptimizer(Node):
    def __init__(self, **kwargs):
        super().__init__('test_optimizer', **kwargs)
        self.requests = []
        self.overrides = {}
        self.create_service(OptimizeRoute, 'optimize_route', self.respond)

    def respond(self, request, response):
        self.requests.append(request)
        response.success = True
        response.method = 'test_echo'
        response.mission_id = request.mission_id
        response.request_id = request.request_id
        response.base_version = request.base_version
        response.cost_revision = request.cost_revision
        response.ordered_step_ids = [step.step_id for step in request.pending_steps]
        for field, value in self.overrides.items():
            setattr(response, field, value)
        return response


if __name__ == '__main__':
    rclpy.init()
    server = MockOptimizer()
    try:
        rclpy.spin(server)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.destroy_node()
        rclpy.try_shutdown()
