# MIT License

# Copyright (c) 2024 Henrik Hose

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from acados_template import AcadosSim, AcadosSimSolver
from wheelbot_dynamics.wheelbot_model import export_wheelbot_ode_model, load_parameter_values
from utils import plot_wheelbot, input_sequence
import numpy as np

def main(use_cython=True, code_reuse=False):

    sim = AcadosSim()
    sim.model = export_wheelbot_ode_model()
    parameters = load_parameter_values("mini_wheelbot_sysid_parameters.json")
    sim.parameter_values = parameters
    
    Tf = 0.02
    nx = sim.model.x.rows()
    N_sim = 40

    # set simulation time
    sim.solver_options.T = Tf
    # set options
    sim.solver_options.integrator_type = 'IRK'
    sim.solver_options.num_stages = 6
    sim.solver_options.num_steps = 3
    sim.solver_options.newton_iter = 10 # for implicit integrator
    sim.solver_options.collocation_type = "GAUSS_RADAU_IIA"

    # create
    if use_cython:
        json_file = f"acados_sim_{sim.model.name}.json"
        if not code_reuse:
            AcadosSimSolver.generate(sim, json_file=json_file)
            AcadosSimSolver.build(
                sim.code_export_directory, with_cython=True
            )
        acados_integrator = AcadosSimSolver.create_cython_solver(json_file)
    else:
        acados_integrator = AcadosSimSolver(sim)

    x0 = np.array(np.deg2rad([20, 1, -1,  0,0,0,  0,0,  0,0]))
    u_seq = input_sequence # should be shape [nu, N_sim-1]

    simX = np.zeros((nx, N_sim))
    simX[:,0] = x0
    
    time_sim = 0

    for i in range(N_sim-1):
        # Note that xdot is only used if an IRK integrator is used
        simX[:, i+1] = acados_integrator.simulate(x=simX[:,i], u=u_seq[:,i])
        time_sim += acados_integrator.get("time_tot")

    print(f"Total simulation time was {time_sim*1000:.3f}ms, that is {time_sim/(N_sim-1)*1000:.3f}ms per integration timestep")
    
    plot_wheelbot([simX], [u_seq], ["simulated_trajectory"])

if __name__ == "__main__":
    main()