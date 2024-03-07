---
layout: default
title: Bill of Materials
nav_order: 5
---

The full component list for our system, with prices at the time of purchase, are lifted below. When building the system, note that two battery modules are required for every rover: one for active discharge to the rover, and a second to concurrently charge at the hub. Additionally, note the use of custom components: many components were 3D printed, and several custom PCBs were created. The associated CAD files can be found at [this link](https://river-lab.github.io/BOOST/CAD.html), and further information on the electronics can be found [at this link](https://river-lab.github.io/BOOST/ELECTRONICS.html).

## Rover

| Top Level         | Level 2          | QTY | Unit Cost | Subtotal      |
| ----------------- | ---------------- | --- | --------- | ------------- |
| Drive Base        | -                | -   | -         | -             |
|                   | Tank Kit         | 1   | $33.99    | $33.99        |
|                   | Motors           | 2   | $15.88    | $31.76        |
| Electronics Plate | -                | -   | -         | -             |
|                   | Motor Driver     | 1   | $2.88     | $2.88         |
|                   | 5V Regulator     | 1   | $4.00     | $4.00         |
|                   | 12V Regulator    | 1   | $10.99    | $10.99        |
|                   | Nucelo           | 1   | $14.90    | $14.90        |
|                   | Wifi Adapter     | 1   | $13.99    | $13.99        |
|                   | GPS Antenna      | 1   | $8.58     | $8.58         |
|                   | Jetson Nano      | 1   | $149.00   | $149.00       |
|                   | Jetson Nano Fan  | 1   | $7.75     | $7.75         |
|                   | Rover PCB        | 1   | $50.00    | $50.00        |
| Camera Assm       | -                | -   | -         | -             |
|                   | T265 Camera      | 1   | $329.00   | $329.00       |
|                   | D435 Camera      | 1   | $314.00   | $314.00       |
|                   | T265 Cable       | 1   | $6.99     | $6.99         |
|                   | D435 Cable       | 1   | $13.79    | $13.79        |
| Battery Reciever  | -                | -   | -         | -             |
|                   | Connector PCB    | 2   | $5.00     | $10.00        |
|                   | Battery Swap PCB | 1   | $12.00    | $12.00        |
|                   | -                | -   | -         | -             |
| **Grand Total**   |                  |     |           | **$1,013.62** |
## Battery Module

| Top Level   | Level 2           | QTY | Unit Cost | Subtotal |
| ----------- | ----------------- | --- | --------- | -------- |
| Electronics |                   |     |           |          |
|             | Battery           | 1   | $53.99    | $53.99   |
|             | Relay             | 1   | $1.60     | $1.60    |
|             | Fuse/Fuse Holder  | 1   | $1.99     | $1.99    |
|             | Regulator         | 1   | $4.00     | $4.00    |
|             | BMS               | 1   | $4.50     | $4.50    |
|             | Connector PCB     | 2   | $5.00     | $10.00   |
|             | XT60 Connector    | 1   | $3.33     | $3.33    |
|             | Balance Connector | 1   | $2.00     | $2.00    |
| **Grand Total** |                   |     |           | **$81.41**   |

## Hub

| Top Level            | Level 2                | QTY | Unit Cost | Subtotal  |
| -------------------- | ---------------------- | --- | --------- | --------- |
| Driven Wheel Assm    | -                      | -   | -         | -         |
|                      | Cim Motor              | 2   | $32.99    | $65.98    |
|                      | Motor Controller       | 2   | $49.99    | $99.98    |
|                      | Bearings               | 4   | $3.99     | $15.96    |
|                      | Shaft Collars          | 4   | $2.99     | $11.96    |
|                      | GearBox and Adapter    | 2   | $85.00    | $170.00   |
|                      | Wheel                  | 2   | $7.99     | $15.98    |
|                      | Shaft                  | 2   | $12.50    | $24.99    |
|                      | Battery                | 1   | $39.50    | $39.50    |
|                      | 40 A Breakers          | 2   | $9.99     | $19.98    |
| Passive Wheel Assm   | -                      | -   | -         | -         |
|                      | Wheel                  | 2   | $7.99     | $15.98    |
|                      | Bolt                   | 2   | $3.63     | $7.26     |
|                      | Nut                    | 2   | $2.23     | $4.46     |
| Charging Electronics | -                      | -   | -         | -         |
|                      | 24v 22Ahr Lipo Battery | 1   | $111.95   | $111.95   |
|                      | Cable Chain            | 1   | $13.88    | $13.88    |
|                      | Fuse/Fuse Holder       | 3   | $1.99     | $5.97     |
|                      | 24v Boost Regulator    | 3   | $29.99    | $89.97    |
|                      | 8 Channel Relay        | 1   | 10.99     | $10.99    |
|                      | 5V Regulator           | 1   | $4.00     | $4.00     |
|                      | DC Voltage Reducer     | 1   | $27.99    | $27.99    |
| Control Electronics  | -                      | -   | -         | -         |
|                      | Wifi Adapter           | 1   | $13.99    | $13.99    |
|                      | GPS Antenna            | 1   | $8.58     | $8.58     |
|                      | Jetson Nano            | 1   | $149.00   | $149.00   |
|                      | Jetson Nano Fan        | 1   | $7.75     | $7.75     |
|                      | Rover PCB              | 1   | $50.00    | $50.00    |
|                      | Nucleo                 | 1   | $32.90    | $32.90    |
|                      | Arduino Nano           | 1   | $5.83     | $5.83     |
|                      | Battery                | 1   | 60.02     | $60.02    |
|                      | LEDs                   | 1   | $21.99    | $21.99    |
|                      | 5V Regulator           | 1   | $4.00     | $4.00     |
|                      | CNC Driver             | 1   | $21.99    | $21.99    |
|                      | Receiver               | 1   | $38.00    | $38.00    |
| Hub Frame            | -                      | -   | -         | -         |
|                      | Top Plate              | 1   | 157.61    | $157.61   |
|                      | Bottom Plate           | 1   | 134.54    | $134.54   |
|                      | 8020 Extrusion         | 1   | $100.00   | $100.00   |
|                      | Corner Brackets        | 6   | $9.50     | $57.00    |
|                      | Angled Brackets        | 2   | $18.73    | $37.46    |
| Slicer Assm          | -                      | -   | -         | -         |
|                      | Linear Rail            | 2   | $29.99    | $59.98    |
|                      | Belt                   | 1   | $8.99     | $8.99     |
|                      | Pulleys                | 2   | $12.87    | $25.74    |
|                      | Nema 23 Motor          | 1   | $20.00    | $20.00    |
| Indexer Assm         | -                      | -   | -         | -         |
|                      | Cartridge Plate        | 1   | $21.02    | $21.02    |
|                      | Nema 23 Motor          | 1   | $20.00    | $20.00    |
|                      | Linear Rail            | 2   | $37.99    | $75.98    |
|                      | Idler Pulley           | 2   | $9.67     | $19.34    |
|                      | Pulley                 | 3   | $10.60    | $31.80    |
|                      | Belt                   | 1   | $8.42     | $8.42     |
|                      | Connector PCB          | 3   | $5.00     | $15.00    |
| Docking Assm         | -                      | -   | -         | -         |
|                      | Connector PCB          | 1   | $5.00     | $5.00     |
|                      | Cable Chain            | 1   | $10.69    | $10.69    |
|                      | Linear Rails/Guides    | 2   | $78.80    | $157.60   |
|                      | Pulleys                | 3   | $14.27    | $42.81    |
|                      | Belt                   | 1   | $15.22    | $15.22    |
|                      | Nema 23 Motor          | 1   | $20.00    | $20.00    |
|                      | Shaft Coupler          | 2   | $4.50     | $8.99     |
|                      | Idler Pulley           | 2   | $9.67     | $19.34    |
| **Grand Total**          |                        |     |           | **$2,243.34** |
