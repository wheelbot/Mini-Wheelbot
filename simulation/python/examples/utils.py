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

import numpy as np
import matplotlib.pyplot as plt

input_sequence = np.array([[
    7.16863230e-02, -2.19887449e-03, -1.03032354e-01, -7.99754754e-02,
    -7.51736739e-03,  3.73970667e-02,  2.78932040e-02,  1.19480389e-02,
    5.04149427e-03,  2.87752798e-03,  2.05901453e-03,  1.54185298e-03,
    1.10553894e-03,  7.09367148e-04,  3.46599073e-04,  1.74089123e-05,
    -2.76907636e-04, -5.35521009e-04, -7.59393363e-04, -1.04580191e-05,
    1.35972571e-04, -7.34172470e-05, -2.84279040e-04, -4.97533320e-04,
    -7.13962177e-04, -9.34304999e-04, -1.15915956e-03, -1.38887471e-03,
    -1.62343091e-03, -1.86230365e-03, -2.10430305e-03, -2.34738146e-03,
    -2.58839709e-03, -2.82281677e-03, -3.04432725e-03, -3.24424413e-03,
    -3.37373970e-03, -3.33552161e-03,  9.90118541e-03],
    [5.00000010e-01,  1.11932453e-01, -5.00000010e-01, -5.00000010e-01,
    -1.49512657e-01,  2.65834309e-01,  1.80153498e-01,  6.41534993e-02,
    1.49805342e-02,  8.54396038e-04, -3.12809724e-03, -4.60706577e-03,
    -5.29701468e-03, -5.60786711e-03, -5.68874070e-03, -5.62021049e-03,
    -5.45532906e-03, -5.23235714e-03, -4.89659338e-03, -2.33803871e-03,
    -1.41103149e-03, -1.10876471e-03, -8.34918670e-04, -5.84838081e-04,
    -3.50720497e-04, -1.24986058e-04,  1.00113562e-04,  3.32865898e-04,
    5.82557265e-04,  8.60057580e-04,  1.17858072e-03,  1.55468965e-03,
    2.00964153e-03,  2.57120357e-03,  3.27611237e-03,  4.17005641e-03,
    5.29552942e-03, -1.17031761e-02,  2.55845289e-08]])


default_subplot_labels_x = [
    "yaw [rad]",
    "roll [rad]",
    "pitch [rad]",
    "yaw_rate [rad/s]",
    "roll_rate [rad/s]",
    "pitch_rate [rad/s]",
    "driving angle [rad]",
    "reaction angle [rad]",
    "driving rate [rad/s]",
    "reaction rate [rad/s]"
]

default_subplot_labels_u = [
    "torque driving [Nm]",
    "torque reaction [Nm]"
]

def plot_wheelbot(data_x, data_u, plot_labels, subplot_labels_x = default_subplot_labels_x, subplot_labels_u = default_subplot_labels_u):
    # Plot each column
    import matplotlib.pyplot as plt
    nx = data_x[0].shape[0]
    nu = data_u[0].shape[0]

    fig_width = 7.2  # inches
    fig_height = fig_width * (16/9)  # maintain 9:16 aspect ratio
    
    fig, axs = plt.subplots(nrows=nx+nu, ncols=1, figsize=(1.5*fig_width, 1.5*fig_height))

    fig.suptitle('Wheelbot')
    for i in range(nx):

        ax = axs[i]

        for idx, dat in enumerate(data_x):
            ax.plot(dat[i,:], label=f"{plot_labels[idx]}", linestyle="dashed")

        ax.set_ylabel(f"{subplot_labels_x[i]}")
        ax.grid()
        ax.legend(loc=1)

    for i in range(nu):

        ax = axs[i+nx]

        for idx, dat in enumerate(data_u):
            ax.step(range(len(dat[i,:])),np.append(dat[i,0], dat[i,:-1]), label=f"{plot_labels[idx]}", linestyle="dashed")

        ax.set_ylabel(f"{subplot_labels_u[i]}")
        ax.grid()
        ax.legend(loc=1)

    plt.xlabel("t [ms]")
    plt.tight_layout(pad=0, w_pad=0, h_pad=0)
    plt.subplots_adjust(left=0.06, right=0.97, bottom=0.04, top=0.95, wspace=0.2, hspace=0.35)
    
    plt.savefig('wheelbot_plot.pdf', format='pdf')
    plt.show()