---
layout: default
title: Electronics
nav_order: 4
---
<h1>Schematics</h1>

This page provides a high-level overview of the electronics of the prototype. Block diagrams for each main system (battery modules, hub, and rover) are provided. Three custom PCBs were created for their function: a rover PCB, a connector PCB, and a power swap PCB. The full design files for these boards are available in KiCad and Altium file formats in [the repository](https://github.com/RIVeR-Lab/BOOST/tree/main/electrical/).

<h2>Battery Module Electronics</h2>
<div style="text-align: center;">
  <figure>
      <img src="./media/battery_module_schematic.png" alt="Battery Module Schematic">
    <figcaption>The battery module is the unit that is exchanged between the minibot and hubbot. The battery module consists of a 6S lipo battery, BMS, custom connector PCBs, as well as protection circuitry. The battery module housing is 3D printed and has design elements for proper alignment and exchange. 
    </figcaption>
  </figure>
</div>

<h2>Hub Electronics</h2>
<div style="text-align: center;">
  <figure>
      <img src="./media/hub_system_diagram.png" alt="Hub Electronics Diagram">
    <figcaption>The hub consists of electronic components for sensing the states of battery modules and circuitry for recharging the 3x battery modules.
    </figcaption>
  </figure>
</div>


<h2>Rover Electronics</h2>
<div style="text-align: center;">
  <figure>
      <img src="./media/rover_electronics.png" alt="Rover Electronics Diagram">
    <figcaption>A simplified block diagram of Rover's electronics.
    </figcaption>
  </figure>
</div>
