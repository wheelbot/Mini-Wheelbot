# Intro
The "joints" of the wheelbot are in order:

1. The yaw angle $\psi$; its derivative is the yaw rate~$\dot{\psi}$.
2. The roll angle $\phi$; its derivative is the roll rate~$\dot{\phi}$.
3. The pitch angle $\theta$; its derivative is the pitch rate~$\dot{\phi}$.
4. The driving wheel angle $q_w$ that is actually doesn't appear in the equations of motions; its derivative is the wheel rotational velocity $\dot{q}_W$; the input torque applied to this joint is $\tau_W$.
5. The reaction wheel angle $q_w$ that is actually doesn't appear in the equations of motions; its derivative is the wheel rotational velocity $\dot{q}_R$; the input torque applied to this joint is $\tau_R$.

The generalized coordinates are: $q=[\psi,\phi,\theta,q_W, q_R]^\top$

The inputs of the wheelbot are: $u=[\tau_W, \tau_R]$

## Continuous time
In continuous time, the dynamics are of the form: $M(q)\ddot{q}+b(q,\dot{q})+g(q,u)=0$.
The mass matrix $M$, coriolis and centripedal terms $b$, and gravitational terms and inputs $g$ are also parametric in the mass and inertia of the wheels W and R, and the body of the robot, and in the wheel radius and distance from wheel rotational axis to body center of mass (assumed to be in the center of the body).

In this repository, you will find functions `fM`,`fb`,`fg` for a generic Wheelbot type of robot alongside specific `fM_WheelbotvXX`, `fb_WheelbotvXX`,`fg_WheelbotvXX` functions, where the mass and inertia parameters of the specific hardware versions are already substituted in.


# Python example
Inside the `python` folder, you find a python package with the dynamics and parameters for the new Mini Wheelbot alongside an acados simulation example.

## Installation
Install acados following the official installation instructions [here](https://docs.acados.org/installation/index.html#linux-mac).
You might need to append acados to your path, you can conveniently do this in your `.bashrc` or manually, whenever you need acados:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/path/to/acados/lib"
export ACADOS_SOURCE_DIR="/path/to/acados"
```

I recommend to proceed with a python virtual environment from here:
```
python -m venv wheelbot-venv
source wheelbot-venv/bin/activate
```

Now install the Python interface for acados by calling:
```
pip install -e /path/to/acados/interfaces/acados_template
```
This should automatically also install casadi.
Now, you are ready to install the `wheelbot-dynamics` package as:
```
pip install -e /path/to/wheelbot-dynamics-functions/python
```

## Running the example
To run the example, you can
```
cd /path/to/wheelbot-dynamics-functions/python/wheelbot-dynamics/examples
python wheelbot_example.py
```
You should see a plot with all states of the wheelbot.
