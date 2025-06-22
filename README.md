# BOOST: Battery-Swapping Multi-Agent System for Sustained Operation of Large Planetary Fleets
[![Image of BOOST system, feature hub bot and mini-rover docked in an outdoor field environment.](https://river-lab.github.io/BOOST/media/boostglamour.jpg)](https://river-lab.github.io/BOOST/)
## Abstract
We propose a novel, heterogeneous multi-agent architecture that miniaturizes rovers by outsourcing power generation to a central hub. By delegating power generation and distribution functions to this hub, the size, weight, power, and cost (SWAP-C) per rover are reduced, enabling efficient fleet scaling. As these rovers conduct mission tasks around the terrain, the hub charges an array of replacement battery modules. When a rover requires charging, it returns to the hub to initiate an autonomous docking sequence and exits with a fully charged battery. This confers an advantage over direct charging methods, such as wireless or wired charging, by replenishing a rover in minutes as opposed to hours, increasing net rover uptime.

This work shares an open-source platform developed to demonstrate battery swapping on unknown field terrain. We detail our design methodologies utilized for increasing system reliability, with a focus on optimization, robust mechanical design, and verification. Optimization of the system is discussed, including the design of passive guide rails through simulation-based optimization methods which increase the valid docking configuration space by 258%. The full system was evaluated during integrated testing, where an average servicing time of 98 seconds was achieved on surfaces with a gradient up to 10°. We conclude by briefly proposing flight considerations for advancing the system toward a space-ready design. In sum, this prototype represents a proof of concept for autonomous docking and battery transfer on field terrain, advancing its Technology Readiness Level (TRL) from 1 to 3.
## Building the system
[CAD models are available](https://river-lab.github.io/BOOST/CAD.html) in a wide variety of data formats; at the time, assembly instructions are not available. [Electronic schematics](https://river-lab.github.io/BOOST/ELECTRONICS.html) and [custom PCBs](electrical/) are both accessible.

## Installing Software
A [software overview](https://river-lab.github.io/BOOST/CODE.html) is provided. Further instructions can be found at [SOFTWARE.md](SOFTWARE.md).

## Links
- [Demo Video](https://www.youtube.com/watch?v=pb5BIy4iOmw)
- [Website](https://river-lab.github.io/BOOST/)
- [Paper (IEEEXplore)](https://ieeexplore.ieee.org/document/10521295)
- [Preprint (ArXiV)](https://arxiv.org/abs/2401.08497)

## Citing this work
```bibtex
@INPROCEEDINGS{10521295,
  author={Holand, Ethan and Homer, Jarrod and Storrer, Alex and Khandeker, Musheeera and Muhlon, Ethan F. and Patel, Maulik and Vainqueur, Ben-oni and Antaki, David and Cooke, Naomi and Wilson, Chloe and Shafai, Bahram and Hanson, Nathaniel and Padır, Taşkın},
  booktitle={2024 IEEE Aerospace Conference}, 
  title={Battery-Swapping Multi-Agent System for Sustained Operation of Large Planetary Fleets}, 
  year={2024},
  volume={},
  number={},
  pages={1-15},
  keywords={Wireless communication;Rails;Prototypes;Optimization methods;Batteries;Outsourcing;Reliability},
  doi={10.1109/AERO58975.2024.10521295}}
```


