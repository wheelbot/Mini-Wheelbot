<!-- ![The Mini Wheelbot](imgs/intro.png) -->
![Teaser](imgs/teaser.png)

# The Mini Wheelbot

The Mini Wheelbot is a **balancing, reaction wheel
unicycle robot** designed as a testbed for learning-based control.
It is an unstable system with highly nonlinear yaw dynamics, non-holonomic driving, and discrete contact switches in a small, powerful, and rugged form factor. 
The Mini Wheelbot can use its wheels to stand up from any initial orientation – enabling automatic environment resets in repetitive experiments.

## Videos
An overview of Wheelbot Hardware is available here:
<!-- [![ICRA 2025 Video](https://img.youtube.com/vi/o1RdiYUH9uY/0.jpg)](https://www.youtube.com/watch?v=o1RdiYUH9uY) -->

## Citation
The Mini Wheelbot is introduced in the paper:
[Henrik Hose, Jan Weissgerber, and Sebastian Trimpe. "The Mini Wheelbot: A Testbed for Learning-based Balancing, Flips, and Articulated Driving", accepted to the IEEE International Conference on Robotics and Automation ICRA (2025).](https://arxiv.org)

Please cite our paper on the Mini Wheelbot:
```
@inproceedings{hose2025mini,
    title={The Mini Wheelbot: A Testbed for Learning-based Balancing, Flips, and Articulated Driving},
    author={Hose, Henrik and Weissgerber, Jan and Trimpe},
    year={2025},
    booktitle={2025 IEEE International Conference on Robotics and Automation (ICRA)}
}
```

## Papers using the Mini Wheelbot Hardware
The following papers use Mini Wheelbot:
- [Henrik Hose, Jan Weissgerber, and Sebastian Trimpe. "The Mini Wheelbot: A Testbed for Learning-based Balancing, Flips, and Articulated Driving", accepted to the IEEE International Conference on Robotics and Automation ICRA (2025).](https://arxiv.org)
- [Hose, Henrik, Paul Brunzema, Alexander von Rohr, Alexander Gräfe, Angela P. Schoellig, and Sebastian Trimpe. "Fine-Tuning of Neural Network Approximate MPC without Retraining via Bayesian Optimization." In CoRL Workshop on Safe and Robust Robot Learning for Operation in the Real World. 2024.](https://openreview.net/pdf?id=lSah6an1Ar)


## Structure of this Repo
This repo contains the following materials:
- **Schematics** of PCBs used on the Mini Wheelbot
- **STLs** of Mini Wheelbot
- **Wheelbot-Lib**: C++20 code running on the Mini Wheelbots Buildroot Linux
- **Scripts**: Collection of Python Scripts used for [system identification](link), etc.
- **Microcontroller**: embedded software running on STM32 microcontrollers on the Mini Wheelbot, using the [modm embedded library builder]().

## Related Repos and Acknowledgements
The Mini Wheelbot's development has been made possible thanks to these open-source projects:
- [modm](https://modm.io/): A modern C++ barebone embedded library generator
- [KiCAD](https://www.kicad.org/): A Cross Platform and Open Source Electronics Design Automation Suite
- [CasADi](https://web.casadi.org/): A symbolic framework for numeric optimization implementing automatic differentiation
- [Buildroot](https://buildroot.org/): A tool to generate embedded Linux systems through cross-compilation




