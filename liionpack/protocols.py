
#
# Experimental protocol
#

import numpy as np


def generate_protocol_from_experiment(experiment, flatten=True):
    """
    Args:
        experiment (pybamm.Experiment):
            The experiment to generate the protocol from.
        flatten (bool):
            Default is True: return all steps in one list otherwise return a
            list of lists for each operating command.
    Returns:
        list:
            a sequence of terminal currents to apply at each timestep
    """
    protocol = []
    for i, op in enumerate(experiment.operating_conditions):
        # op["Terminal voltage [V]"][0] = 3 * 2 # initilising with "nominal voltage"
        proto = []

        t = op["time"]
        dt = op["period"]
        # if t % dt != 0:
        #     raise ValueError("Time must be an integer multiple of the period")

        typ = op["type"]
        if typ in ["power"]:
            if typ == "power":
                if "Power input [W]" in op.keys():
                    
                    P = op["Power input [W]"]
                    I = op["Current input [A]"]
                    V = op["Terminal voltage [V]"]

                    #V = 3.9
                    if t == 0:
                        P = V * I
                        V[0] = P[0]/I[0]
                    for i in t[1:end]:
                        I[i] = P[i]/V[i-1]
                    proto.extend([I] * int(t / dt))
                    if i == 0:
                        # Include initial state when not drive cycle, first op
                        proto = [proto[0]] + proto
                # elif "dc_data" in op.keys():
                #     dc_data = op["dc_data"]
                    
                #     proto.extend(dc_data_I[:, 1].tolist())
                    
        # if typ not in ["current"]:
        #     raise ValueError("Only constant current operations are supported")
        # else:
        if typ in ["current"]:
            if typ == "current":
                if "Current input [A]" in op.keys():
                    I = op["Current input [A]"]
                    proto.extend([I] * int(t / dt))
                    if i == 0:
                        # Include initial state when not drive cycle, first op
                        proto = [proto[0]] + proto
                elif "dc_data" in op.keys():
                    dc_data = op["dc_data"]
                    proto.extend(dc_data[:, 1].tolist())

        if flatten:
            protocol.extend(proto)
        else:
            protocol.append(proto)

    return protocol
'''
        def external_circuit_function(variables):
            I = variables["Current [A]"]
            V = variables["Terminal voltage [V]"]
            return V + I - pybamm.FunctionParameter("Function", {"Time [s]": pybamm.t})

    def get_fundamental_variables(self):
        param = self.param
        # Current is a variable
        i_var = pybamm.Variable("Current density variable")
        if self.control in ["algebraic", "differential without max"]:
            i_cell = i_var
        elif self.control == "differential with max":
            i_input = (
                pybamm.FunctionParameter(
                    "CCCV current function [A]",
                    {"Time [s]": pybamm.t * param.timescale},
                )
                / param.I_typ
                * pybamm.sign(param.I_typ)
            )
            i_cell = pybamm.maximum(i_var, i_input)

        # Update derived variables
        I = i_cell * abs(param.I_typ)
        i_cell_dim = I / (param.n_electrodes_parallel * param.A_cc)

        variables = {
            "Current density variable": i_var,
            "Total current density": i_cell,
            "Total current density [A.m-2]": i_cell_dim,
            "Current [A]": I,
            "C-rate": I / param.Q,
        }

        # Add discharge capacity variable
        variables.update(super().get_fundamental_variables())

        return variables

class PowerFunctionControl(FunctionControl):
    """External circuit with power control."""

    def __init__(self, param, options, control="algebraic"):
        super().__init__(param, self.constant_power, options, control=control)

    def constant_power(self, variables):
        I = variables["Current [A]"]
        V = variables["Terminal voltage [V]"]
        P = V * I
        P_applied = pybamm.FunctionParameter(
            "Power function [W]", {"Time [s]": pybamm.t * self.param.timescale}
        )
        if self.control == "algebraic":
            return P - P_applied
        else:
            # Multiply by the time scale so that the overshoot only lasts a few seconds
            K_P = 0.01 * self.param.timescale
            return -K_P * (P - P_applied)
'''