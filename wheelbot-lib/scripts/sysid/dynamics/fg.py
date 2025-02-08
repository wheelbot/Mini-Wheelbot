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

from casadi import sin, cos, vertcat
def fg(phi, theta, tau_W, tau_R, m_W, m_B, m_R, r_W, l_WB):
    return vertcat(
        0,
        -(981*sin(phi)*(m_B*r_W+m_R*r_W+m_W*r_W+l_WB*m_B*cos(theta)+2*l_WB*m_R*cos(theta)))/100,
        tau_W-(981*l_WB*m_B*cos(phi)*sin(theta))/100-(981*l_WB*m_R*cos(phi)*sin(theta))/50,
        -tau_W,
        -tau_R
    )
