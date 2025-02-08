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

from casadi import sin, cos, vertcat, horzcat
def fM(phi, theta, m_W, m_B, m_R, I_Wx, I_Wy, I_Wz, I_Bx, I_By, I_Bz, I_Rx, I_Ry, I_Rz, r_W, l_WB):
    return vertcat(
           horzcat(l_WB**2*m_B-(cos(phi)**2-1)*(I_By+I_Ry+I_Wy)+4*l_WB**2*m_R+m_B*r_W**2+m_R*r_W**2+m_W*r_W**2+I_Wz*cos(phi)**2-cos(phi)**2*(cos(theta)**2-1)*(I_Bx+I_Rx)+cos(phi)**2*cos(theta)**2*(I_Bz+I_Rz)-m_B*r_W**2*cos(phi)**2-m_R*r_W**2*cos(phi)**2-m_W*r_W**2*cos(phi)**2-l_WB**2*m_B*cos(phi)**2*cos(theta)**2-4*l_WB**2*m_R*cos(phi)**2*cos(theta)**2+2*l_WB*m_B*r_W*cos(theta)+4*l_WB*m_R*r_W*cos(theta)-2*l_WB*m_B*r_W*cos(phi)**2*cos(theta)-4*l_WB*m_R*r_W*cos(phi)**2*cos(theta), -cos(phi)*sin(theta)*(I_Bx*cos(theta)-I_Bz*cos(theta)+I_Rx*cos(theta)-I_Rz*cos(theta)+l_WB**2*m_B*cos(theta)+4*l_WB**2*m_R*cos(theta)+l_WB*m_B*r_W+2*l_WB*m_R*r_W), sin(phi)*(I_By+I_Ry+l_WB**2*m_B+4*l_WB**2*m_R+l_WB*m_B*r_W*cos(theta)+2*l_WB*m_R*r_W*cos(theta)), sin(phi)*(I_Wy+m_B*r_W**2+m_R*r_W**2+m_W*r_W**2+l_WB*m_B*r_W*cos(theta)+2*l_WB*m_R*r_W*cos(theta)), -I_Rx*cos(phi)*sin(theta)),
           horzcat(-cos(phi)*sin(theta)*(I_Bx*cos(theta)-I_Bz*cos(theta)+I_Rx*cos(theta)-I_Rz*cos(theta)+l_WB**2*m_B*cos(theta)+4*l_WB**2*m_R*cos(theta)+l_WB*m_B*r_W+2*l_WB*m_R*r_W), I_Bz+I_Rz+I_Wx+m_B*r_W**2+m_R*r_W**2+m_W*r_W**2+I_Bx*cos(theta)**2-I_Bz*cos(theta)**2+I_Rx*cos(theta)**2-I_Rz*cos(theta)**2+l_WB**2*m_B*cos(theta)**2+4*l_WB**2*m_R*cos(theta)**2+2*l_WB*m_B*r_W*cos(theta)+4*l_WB*m_R*r_W*cos(theta), 0, 0, I_Rx*cos(theta)),
           horzcat(sin(phi)*(I_By+I_Ry+l_WB**2*m_B+4*l_WB**2*m_R+l_WB*m_B*r_W*cos(theta)+2*l_WB*m_R*r_W*cos(theta)), 0, I_By+I_Ry+l_WB**2*m_B+4*l_WB**2*m_R, l_WB*r_W*cos(theta)*(m_B+2*m_R), 0),
           horzcat(sin(phi)*(I_Wy+m_B*r_W**2+m_R*r_W**2+m_W*r_W**2+l_WB*m_B*r_W*cos(theta)+2*l_WB*m_R*r_W*cos(theta)), 0, l_WB*r_W*cos(theta)*(m_B+2*m_R), I_Wy+m_B*r_W**2+m_R*r_W**2+m_W*r_W**2, 0),
           horzcat(-I_Rx*cos(phi)*sin(theta), I_Rx*cos(theta), 0, 0, I_Rx)
    )
