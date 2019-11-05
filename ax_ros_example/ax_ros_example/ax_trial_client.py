from ax.service.ax_client import AxClient
import numpy as np
import rclpy
from rclpy.node import Node
from ax_ros_interfaces.srv import RunAxTrial
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters

class AxTrialClient(Node):
    def __init__(self, name = "ROS2 Bayesian Optimization"):
        super().__init__('ax_test_server')
        self.cli = self.create_client(RunAxTrial, 'simple_ax_trial')
        self.param_cli = self.create_client(SetParameters, '/ax_test_server/set_parameters')

        self.ax = AxClient()
        self.name = name
        self.parameters = []
        self.result = []
        while not self.param_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('parameter service not available, waiting again...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RunAxTrial.Request()

    def create_ax_experiment(self):
        self.ax.create_experiment(
            name=self.name,
            parameters=[
                {
                    "name": "x",
                    "type": "range",
                    "bounds": [-10.0, 10.0],
                    "value_type": "float",  # Optional, defaults to inference from type of "bounds".
                    "log_scale": False,  # Optional, defaults to False.
                }
            ],
            objective_name="squared",
            minimize=True,  # Optional, defaults to False.
        )

    # def get_ros_parameters(self)

    # def convert_params_to_ax(self)

    def setup_trial(self):
        params = Parameter(name="x", value = ParameterValue(type=3, double_value=self.parameters.get(f"x")))
        request = SetParameters.Request()
        request.parameters = [params]
        future_params = self.param_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future_params)

    def invoke_trial(self):
        future_trial = response = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future_trial)
        response = future_trial.result()
        self.result = response.result

    def evaluate_trial(self):
        return {"squared": (self.result[0], 0.0)}

    def run_experiment(self):
        self.create_ax_experiment()
        for i in range(25):
            # Get next parameter arm candidates
            self.parameters, trial_index = self.ax.get_next_trial()

            # Setup next trial
            self.setup_trial()

            # Start trial run task
            self.invoke_trial()

            # Local evaluation here can be replaced with deployment to external system.
            self.ax.complete_trial(trial_index=trial_index, raw_data=self.evaluate_trial())

def main(args=None):
    rclpy.init(args=args)

    ax_trial_client = AxTrialClient()
    ax_trial_client.run_experiment()
    ax_trial_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()