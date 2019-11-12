from ax.service.ax_client import AxClient
import numpy as np
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from ax_ros_interfaces.srv import RunAxTrial
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import IntegerRange
from rclpy.parameter import Parameter
from ax import Arm
from ax.utils.notebook.plotting import render

class AxTrialClient(Node):
    def __init__(self, name = "ROS2 Bayesian Optimization"):
        super().__init__('ax_test_client')
        self.cli = self.create_client(RunAxTrial, '/simple_ax_server_cpp/trial_request')
        self.param_cli = self.create_client(SetParameters, '/simple_ax_server_cpp/set_parameters')
        self.param_clients = []
        self.parameters_setup = []
        self.ax = AxClient()
        self.name = name
        self.parameters = []
        self.result = []
        self.status_quo = None
        self.objective_name = self.declare_parameter("objective_name", "ax_ros_objective").value
        self.experiment_name = self.declare_parameter("experiment_name", "ax_rox_experiment").value
        self.num_trials = self.declare_parameter("num_trials", 25).value

        while not self.param_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('parameter service not available, waiting again...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RunAxTrial.Request()
        
        with self.handle as capsule:
            self._yaml_parameter_overrides = _rclpy.rclpy_get_param_overrides(Parameter, capsule)
            self._yaml_parameter_descriptor_overrides = _rclpy.rclpy_get_param_descriptor_overrides(ParameterDescriptor, IntegerRange, FloatingPointRange, capsule)   
        
        if '/' + self.get_name() in self._yaml_parameter_overrides.keys():
            self._yaml_parameter_overrides.pop('/' + self.get_name())

        if '/' + self.get_name() in self._yaml_parameter_descriptor_overrides.keys():
            self._yaml_parameter_descriptor_overrides.pop('/' + self.get_name())

    def create_ax_experiment(self):
        self.create_ros_param_clients()
        self.convert_params_to_ax()
        self.ax.create_experiment(
            name=self.experiment_name,
            parameters=self.parameters_setup,
            objective_name=self.objective_name,
            minimize=True,  # Optional, defaults to False.
            status_quo = self.status_quo,
        )

    def get_ros_param_descriptors(self):
        self.ros_param_descriptors = []
        for descriptor in self._yaml_parameter_descriptor_overrides.values():         
            for value in descriptor.values():
                self.ros_param_descriptors.append(value)
        print(self.ros_param_descriptors)

    def get_ros_params(self):
        self.ros_params = []
        # Throw out parameters not associated with descriptor
        for node, param in self._yaml_parameter_overrides.items():
            if node not in self._yaml_parameter_descriptor_overrides.keys():
                continue
            for name, value in param.items():
                if name not in self._yaml_parameter_descriptor_overrides[node].keys():
                    continue
                self.ros_params.append(value)
        print(self.ros_params)

    def create_ros_param_clients(self):
        for node in self._yaml_parameter_descriptor_overrides:
            client = self.create_client(SetParameters, node + '/set_parameters')
            self.param_clients.append(client)
        
    def set_ros_params(self):
        for param_descriptors, client in zip(self._yaml_parameter_descriptor_overrides.values(), self.param_clients):
            request = SetParameters.Request()
            for name in param_descriptors.keys():
                request.parameters.append(Parameter(name = name, value = self.parameters.get(name)).to_parameter_msg())
            future_params = client.call_async(request)
            rclpy.spin_until_future_complete(self, future_params)

    def convert_params_to_ax(self):
        self.get_ros_param_descriptors()
        # only support range parameters
        for param_descriptor in self.ros_param_descriptors:
            if param_descriptor.floating_point_range != []:
                ax_parameter = {
                    "name": param_descriptor.name,
                    "type": "range",
                    "bounds": [param_descriptor.floating_point_range[0].from_value, param_descriptor.floating_point_range[0].to_value],
                    "value_type": "float",
                }
                self.parameters_setup.append(ax_parameter)
            elif param_descriptor.integer_range != []:
                ax_parameter = {
                    "name": param_descriptor.name,
                    "type": "range",
                    "bounds": [param_descriptor.integer_range[0].from_value, param_descriptor.integer_range[0].to_value],
                    "value_type": "int",
                }
                self.parameters_setup.append(ax_parameter)
            else:
                self.get_logger().info("Only 'double' or 'int' parameters are currently supported for ax_ros parameter optimization")

        self.get_ros_params()
        if self.ros_params != []:
            self.get_status_quo()


    def get_status_quo(self):
        # only support range parameters
        self.status_quo = dict()
        for param in self.ros_params:
            if param.type_ == Parameter.Type.DOUBLE:
                self.status_quo[param.name] = param._value
            elif param.type_ == Parameter.Type.INTEGER:
                self.status_quo[param.name] = param._value
            else:
                self.get_logger().info("Only 'double' or 'int' parameters are currently supported for ax_ros parameter optimization")

    def setup_trial(self):
        self.set_ros_params()

    def invoke_trial(self):
        future_trial = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future_trial)
        response = future_trial.result()
        self.result = response.result

    def evaluate_trial(self):
        return {self.objective_name: (self.result[0], 0.0)}

    def run_experiment(self):
        # Run status quo
        if self.status_quo is not None:
            self.parameters, trial_index = self.ax.attach_trial(parameters=self.status_quo)
            self.setup_trial()
            self.invoke_trial()
            self.ax.complete_trial(trial_index=trial_index, raw_data=self.evaluate_trial())


        # Run remaining trials
        for i in range(self.num_trials):
            # Get next parameter arm candidates
            self.parameters, trial_index = self.ax.get_next_trial()

            # Setup next trial
            self.setup_trial()

            # Start trial run task
            self.invoke_trial()

            # Local evaluation here can be replaced with deployment to external system.
            self.ax.complete_trial(trial_index=trial_index, raw_data=self.evaluate_trial())
        self.get_logger().info("Best parameters are:")
        best_parameters, values = self.ax.get_best_parameters()
        print(best_parameters)
        means, covariances = values
        print(means)
        render(self.ax.get_optimization_trace())
def main(args=None):
    rclpy.init(args=args)

    ax_trial_client = AxTrialClient()
    ax_trial_client.run_experiment()
    ax_trial_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()