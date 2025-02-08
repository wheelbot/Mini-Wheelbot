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

import pandas as pd
import numpy as np

from dynamics.wheelbot_model import export_wheelbot_ode_model

from casados_integrator import CasadosIntegrator
from acados_template import AcadosSim
from casadi import *
from datetime import datetime

import json

from datasets import *

def create_casados_integrator(model, integrator_opts, dt=0.1, use_cython=True, code_reuse=False):
    sim = AcadosSim()
    sim.model = model

    # set simulation time
    sim.solver_options.T = dt

    # set options
    sim.solver_options.sens_forw = True
    sim.solver_options.sens_algebraic = False
    sim.solver_options.sens_hess = True
    sim.solver_options.sens_adj = True

    sim.solver_options.integrator_type = "IRK"
    sim.solver_options.num_stages = integrator_opts["num_stages"]
    sim.solver_options.num_steps = integrator_opts["num_steps"]
    sim.solver_options.newton_iter = integrator_opts["newton_iter"]

    if integrator_opts["collocation_scheme"] == "radau":
        sim.solver_options.collocation_type = "GAUSS_RADAU_IIA"
    elif integrator_opts["collocation_scheme"] == "legendre":
        sim.solver_options.collocation_type = "GAUSS_LEGENDRE"
    else:
        raise Exception(
            "integrator_opts['collocation_scheme'] must be radau or legendre."
        )

    sim.solver_options.newton_tol = (
        integrator_opts["tol"] / integrator_opts["num_steps"]
    )
    sim.code_export_directory = f'c_generated_code_{model.name}_{sim.solver_options.integrator_type}'

    casados_integrator = CasadosIntegrator(sim, use_cython=use_cython, code_reuse=code_reuse)
    return casados_integrator


def plot_wheelbot_full_state(measured_data_x, measured_data_u, fitted_data_x, fitted_data_u, subplot_labels_x, subplot_labels_u, plot_labels):
    # Plot each column
    import matplotlib.pyplot as plt
    nx = measured_data_x[0].shape[0]
    nu = measured_data_u[0].shape[0]

    fig_width = 7.2  # inches
    fig_height = fig_width * (16/9)  # maintain 9:16 aspect ratio

    fig, axs = plt.subplots(nrows=nx+nu, ncols=1, figsize=(1.5*fig_width, 1.5*fig_height))

    fig.suptitle('Wheelbot SysID')
    for i in range(nx):

        ax = axs[i]

        for idx, dat in enumerate(measured_data_x):
            ax.plot(dat[i,:], label=f"measured {plot_labels[idx]}", linestyle="dashed")

        ax.set_prop_cycle(None)

        for idx, dat in enumerate(fitted_data_x):
            ax.plot(dat[i,:], label=f"fitted {plot_labels[idx]}")

        ax.set_ylabel(f"{subplot_labels_x[i]}")
        ax.grid()
        ax.legend(loc=1)

    for i in range(nu):

        ax = axs[i+nx]

        for idx, dat in enumerate(measured_data_u):
            ax.step(range(len(dat[i,:])),np.append(dat[i,0], dat[i,:-1]), label=f"measured {plot_labels[idx]}", linestyle="dashed")

        ax.set_prop_cycle(None)

        for idx, dat in enumerate(fitted_data_u):
            ax.step(range(len(dat[i,:])), np.append(dat[i,0], dat[i,:-1]), label=f"fitted {plot_labels[idx]}")

        ax.set_ylabel(f"{subplot_labels_u[i]}")
        ax.grid()
        ax.legend(loc=1)

    plt.xlabel("t [ms]")
    plt.tight_layout(pad=0, w_pad=0, h_pad=0)
    plt.subplots_adjust(left=0.06, right=0.97, bottom=0.04, top=0.95, wspace=0.2, hspace=0.35)

    plt.savefig('wheelbot_sysid_plot.pdf', format='pdf')
    plt.show()


df, start_index = import_dataset_1()

print(f"Start Index is {start_index}")
dT = 10e-3
N_data = 50

selected_columns_x = ['/q_yrp/yaw',         '/q_yrp/roll',       '/q_yrp/pitch',
                    '/dq_yrp/yaw_vel',    '/dq_yrp/roll_vel',  '/dq_yrp/pitch_vel',
                    '/q_DR/drive_wheel',  '/q_DR/reaction_wheel',
                    '/dq_DR/drive_wheel', '/dq_DR/reaction_wheel']
selected_columns_u = ['/tau_DR_command/drive_wheel', '/tau_DR_command/reaction_wheel']

def select_data(start_index):
    selected_df = (df.iloc[start_index:start_index+int(N_data*10)]).iloc[::10]

    selected_data = selected_df[selected_columns_x]
    measured_data_x = selected_data.to_numpy().T

    selected_data = selected_df[selected_columns_u]
    measured_data_u = selected_data.to_numpy().T

    return measured_data_x, measured_data_u


datasets = [select_data(idx) for idx in start_index]


m_W_default = 0.129
m_B_default = 0.312
m_R_default = 0.129

I_Wx_default =  53e-6
I_Wy_default = 100e-6
I_Wz_default =  53e-6

I_Bx_default = 220e-6
I_By_default = 300e-6
I_Bz_default = 190e-6

I_Rx_default = 100e-6
I_Ry_default =  53e-6
I_Rz_default =  53e-6

r_W_default = 0.032
l_WB_default = 0.065

import casadi as cs

opti = cs.Opti()
cost = 0
eps_cost = 1e-3

pitch_bias = opti.variable()
opti.subject_to(pitch_bias==0)
opti.set_initial(pitch_bias, 0)
cost = cost + eps_cost*(pitch_bias)**2

roll_bias = opti.variable()
opti.subject_to(roll_bias==0)
opti.set_initial(roll_bias, 0)
cost = cost + eps_cost*(roll_bias)**2

motor_constant_scaling = opti.variable()
opti.subject_to(0.5 <= motor_constant_scaling)
opti.subject_to(motor_constant_scaling <= 1.5)
opti.subject_to(motor_constant_scaling == 1)
opti.set_initial(motor_constant_scaling, 1)
cost = cost + eps_cost*(motor_constant_scaling)**2

u_bound = 0.001 # 2 percent

m_WR = opti.variable()
opti.subject_to(0.8*m_W_default <= m_WR)
opti.subject_to(m_WR <= 1.2*m_W_default)
opti.set_initial(m_WR, m_W_default)
cost = cost + eps_cost*(m_WR-m_W_default)**2

m_B = opti.variable()
opti.subject_to(0.5*m_B_default <= m_B)
opti.subject_to(m_B <= 2*m_B_default)
opti.set_initial(m_B, m_B_default)
cost = cost + eps_cost*(m_B-m_B_default)**2

I_Wxz_Ryz = opti.variable()
opti.subject_to(0.8*I_Wx_default <= I_Wxz_Ryz)
opti.subject_to(I_Wxz_Ryz <= 1.2*I_Wx_default)
opti.set_initial(I_Wxz_Ryz, I_Wx_default)
cost = cost + eps_cost*(I_Wxz_Ryz-I_Wx_default)**2

I_Wy_Rx = opti.variable()
opti.subject_to(0.8*I_Wy_default <= I_Wy_Rx)
opti.subject_to(I_Wy_Rx <= 1.2*I_Wy_default)
opti.set_initial(I_Wy_Rx, I_Wy_default)
cost = cost + eps_cost*(I_Wy_Rx-I_Wy_default)**2

I_Bx = opti.variable()
opti.subject_to(0.5*I_Bx_default <= I_Bx)
opti.subject_to(I_Bx <= 1.5*I_Bx_default)
opti.set_initial(I_Bx, I_Bx_default)
cost = cost + eps_cost*(I_Bx-I_Bx_default)**2

I_By = opti.variable()
opti.subject_to( 0.5*I_By_default <= I_By)
opti.subject_to(I_By <= 1.5*I_By_default)
opti.set_initial(I_By, I_By_default)
cost = cost + eps_cost*(I_By-I_By_default)**2

I_Bz = opti.variable()
opti.subject_to(0.5*I_Bz_default <= I_Bz)
opti.subject_to(I_Bz <= 1.5*I_Bz_default)
opti.set_initial(I_Bz, I_Bz_default)
cost = cost + eps_cost*(I_Bz-I_Bz_default)**2

r_W = opti.variable()
opti.subject_to(0.95*r_W_default <= r_W)
opti.subject_to(r_W <= 1.05*r_W_default )
opti.set_initial(r_W, r_W_default)
opti.subject_to(r_W==r_W_default)
cost = cost + eps_cost*(r_W-r_W_default)**2

l_WB = opti.variable()
opti.subject_to(0.95*l_WB_default <= l_WB)
opti.subject_to(l_WB <= 1.05*l_WB_default)
opti.set_initial(l_WB, l_WB_default)
opti.subject_to(l_WB==l_WB_default)
cost = cost + eps_cost*(l_WB-l_WB_default)**2

fric_magn_default = 0.005
fric_magn = opti.variable()
opti.subject_to(fric_magn>=0.001*fric_magn_default)
opti.set_initial(fric_magn, fric_magn_default)
cost = cost + eps_cost*(fric_magn-fric_magn_default)**2

fric_slope_default = 50
fric_slope = opti.variable()
opti.subject_to(fric_slope>=0.001*fric_slope_default)
opti.set_initial(fric_slope, fric_slope_default)
cost = cost + eps_cost*(fric_slope-fric_slope_default)**2

N_datasets = len(datasets)
x_var_all_datasets = []
u_var_all_datasets = []


model = export_wheelbot_ode_model(True)
integrator_opts = {
    "type": "implicit",
    "collocation_scheme": "radau",
    "num_stages": 6,
    "num_steps": 3,
    "newton_iter": 10,
    "tol": 1e-8,
}
casados_integrator = create_casados_integrator(
    model, integrator_opts, dt=dT, use_cython=True
    )


for idx, ds in enumerate(datasets):
    measured_data_x, measured_data_u = ds
    x_var = opti.variable(10, N_data)
    u_var = opti.variable(2, N_data)

    x_var_all_datasets.append(x_var)
    u_var_all_datasets.append(u_var)

    opti.set_initial(x_var[:,:], measured_data_x)
    opti.set_initial(u_var[:,:], measured_data_u)

    for i in range(N_data-1):
        x = x_var[:,i] + cs.vertcat(0,roll_bias,pitch_bias,0,0,0,0,0,0,0)
        tau_W = motor_constant_scaling*u_var[0,i]
        tau_R = motor_constant_scaling*u_var[1,i]
        u = cs.vertcat(tau_W, tau_R, m_WR, m_B, I_Wxz_Ryz, I_Wy_Rx, I_Bx, I_By, I_Bz, r_W, l_WB, fric_magn, fric_slope)
        x_next = casados_integrator(x,u)
        opti.subject_to( x_var[:,i+1] == x_next )

    Q_mat = 2 * np.diag([100,1,1,10,10,10,0.1,0.1,0.1,0.1])
    R_mat = 2 * np.diag([100,100])
    for j in range(0,N_data):
        x = x_var[:,j] + cs.vertcat(0,roll_bias,pitch_bias,0,0,0,0,0,0,0)
        x_err = (x-measured_data_x[:,j])
        u_err = (u_var[:,j]-measured_data_u[:,j])
        opti.subject_to( -u_bound <= u_err)
        opti.subject_to( u_err <= u_bound)
        cost = cost + x_err.T@Q_mat@x_err + u_err.T@R_mat@u_err

opti.minimize(cost)
opts = {
        "ipopt.max_iter":5000,
        }


opti.solver("ipopt",opts)
sol = opti.solve()

fitted_m_WR = opti.value(m_WR)
fitted_m_B = opti.value(m_B)
fitted_I_Wxz_Ryz = opti.value(I_Wxz_Ryz)
fitted_I_Wy_Rx = opti.value(I_Wy_Rx)
fitted_I_Bx = opti.value(I_Bx)
fitted_I_By = opti.value(I_By)
fitted_I_Bz = opti.value(I_Bz)
fitted_r_W = opti.value(r_W)
fitted_l_WB = opti.value(l_WB)
fitted_fric_magn = opti.value(fric_magn)
fitted_fric_slope = opti.value(fric_slope)

fitted_roll_bias = opti.value(roll_bias)
fitted_pitch_bias = opti.value(pitch_bias)
fitted_motor_constant_scaling = opti.value(motor_constant_scaling)
fitted_cost = opti.value(cost)

print(f"{fitted_cost=}")
print(f"{fitted_roll_bias=}")
print(f"{fitted_pitch_bias=}")
print(f"{fitted_motor_constant_scaling=}")


print(f"\nIdentified parameters are:")
print(f"\t{fitted_m_WR=}, {m_W_default=}")
print(f"\t{fitted_m_B=}, {m_B_default=}")
print(f"\t{fitted_I_Wxz_Ryz=}, {I_Wx_default=}")
print(f"\t{fitted_I_Wy_Rx=}, {I_Wy_default=}")
print(f"\t{fitted_I_Bx=}, {I_Bx_default=}")
print(f"\t{fitted_I_By=}, {I_By_default=}")
print(f"\t{fitted_I_Bz=}, {I_Bz_default=}")
print(f"\t{fitted_r_W=}, {r_W_default=}")
print(f"\t{fitted_l_WB=}, {l_WB_default=}")
print(f"\t{fitted_fric_magn=}, {fric_magn_default=}")
print(f"\t{fitted_fric_slope=}, {fric_slope_default=}")
print(f"\n")


results = {
    "m_WR": fitted_m_WR,
    "m_B": fitted_m_B,
    "I_Wxz_Ryz": fitted_I_Wxz_Ryz,
    "I_Wy_Rx": fitted_I_Wy_Rx,
    "I_Bx": fitted_I_Bx,
    "I_By": fitted_I_By,
    "I_Bz": fitted_I_Bz,
    "r_W": fitted_r_W,
    "l_WB": fitted_l_WB,
    "fric_magn": fitted_fric_magn,
    "fric_slope": fitted_fric_slope,
}
with open('system_parameters.json', 'w') as f:
    json.dump(results, f)



fitted_data_x = [opti.value(x_var) for x_var in x_var_all_datasets]
fitted_data_u = [opti.value(u_var) for u_var in u_var_all_datasets]
plot_wheelbot_full_state(
    [measured_data_x for measured_data_x, measured_data_u in datasets],
    [measured_data_u for measured_data_x, measured_data_u in datasets],
    fitted_data_x,
    fitted_data_u,
    selected_columns_x,
    selected_columns_u,
    [f"run {idx}" for idx, _ in enumerate(datasets)])

import pickle
data = {
    "measured_data_x": [measured_data_x for measured_data_x, measured_data_u in datasets],
    "measured_data_u": [measured_data_u for measured_data_x, measured_data_u in datasets],
    "fitted_data_x": fitted_data_x,
    "fitted_data_u": fitted_data_u,
    "selected_columns_x": selected_columns_x,
    "selected_columns_u": selected_columns_u,
    "dT": dT
}
with open('results.pkl', 'wb') as file:
    pickle.dump(data, file)

for idx, _ in enumerate(datasets):
    plot_wheelbot_full_state(
        [[measured_data_x for measured_data_x, measured_data_u in datasets][idx]],
        [[measured_data_u for measured_data_x, measured_data_u in datasets][idx]],
        [fitted_data_x[idx]],
        [fitted_data_u[idx]],
        selected_columns_x,
        selected_columns_u,
        [f"run {idx}"])


