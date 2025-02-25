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

function M(ϕ, θ, m_W, m_B, m_R, I_Wx, I_Wy, I_Wz, I_Bx, I_By, I_Bz, I_Rx, I_Ry, I_Rz, r_W, l_WB)
[l_WB^2*m_B-(cos(ϕ)^2-1)*(I_By+I_Ry+I_Wy)+4*l_WB^2*m_R+m_B*r_W^2+m_R*r_W^2+m_W*r_W^2+I_Wz*cos(ϕ)^2-cos(ϕ)^2*(cos(θ)^2-1)*(I_Bx+I_Rx)+cos(ϕ)^2*cos(θ)^2*(I_Bz+I_Rz)-m_B*r_W^2*cos(ϕ)^2-m_R*r_W^2*cos(ϕ)^2-m_W*r_W^2*cos(ϕ)^2-l_WB^2*m_B*cos(ϕ)^2*cos(θ)^2-4*l_WB^2*m_R*cos(ϕ)^2*cos(θ)^2+2*l_WB*m_B*r_W*cos(θ)+4*l_WB*m_R*r_W*cos(θ)-2*l_WB*m_B*r_W*cos(ϕ)^2*cos(θ)-4*l_WB*m_R*r_W*cos(ϕ)^2*cos(θ) -cos(ϕ)*sin(θ)*(I_Bx*cos(θ)-I_Bz*cos(θ)+I_Rx*cos(θ)-I_Rz*cos(θ)+l_WB^2*m_B*cos(θ)+4*l_WB^2*m_R*cos(θ)+l_WB*m_B*r_W+2*l_WB*m_R*r_W) sin(ϕ)*(I_By+I_Ry+l_WB^2*m_B+4*l_WB^2*m_R+l_WB*m_B*r_W*cos(θ)+2*l_WB*m_R*r_W*cos(θ)) sin(ϕ)*(I_Wy+m_B*r_W^2+m_R*r_W^2+m_W*r_W^2+l_WB*m_B*r_W*cos(θ)+2*l_WB*m_R*r_W*cos(θ)) -I_Rx*cos(ϕ)*sin(θ);-cos(ϕ)*sin(θ)*(I_Bx*cos(θ)-I_Bz*cos(θ)+I_Rx*cos(θ)-I_Rz*cos(θ)+l_WB^2*m_B*cos(θ)+4*l_WB^2*m_R*cos(θ)+l_WB*m_B*r_W+2*l_WB*m_R*r_W) I_Bz+I_Rz+I_Wx+m_B*r_W^2+m_R*r_W^2+m_W*r_W^2+I_Bx*cos(θ)^2-I_Bz*cos(θ)^2+I_Rx*cos(θ)^2-I_Rz*cos(θ)^2+l_WB^2*m_B*cos(θ)^2+4*l_WB^2*m_R*cos(θ)^2+2*l_WB*m_B*r_W*cos(θ)+4*l_WB*m_R*r_W*cos(θ) 0 0 I_Rx*cos(θ);sin(ϕ)*(I_By+I_Ry+l_WB^2*m_B+4*l_WB^2*m_R+l_WB*m_B*r_W*cos(θ)+2*l_WB*m_R*r_W*cos(θ)) 0 I_By+I_Ry+l_WB^2*m_B+4*l_WB^2*m_R l_WB*r_W*cos(θ)*(m_B+2*m_R) 0;sin(ϕ)*(I_Wy+m_B*r_W^2+m_R*r_W^2+m_W*r_W^2+l_WB*m_B*r_W*cos(θ)+2*l_WB*m_R*r_W*cos(θ)) 0 l_WB*r_W*cos(θ)*(m_B+2*m_R) I_Wy+m_B*r_W^2+m_R*r_W^2+m_W*r_W^2 0;-I_Rx*cos(ϕ)*sin(θ) I_Rx*cos(θ) 0 0 I_Rx]
end