# Bill of Materials
** This list is work in progress and not yet complete ** 

## CNC Parts
| Item | Description | Qty | Vendor |
| ---- | ---- | --- | --- |
| [Frame](mechanical/step/frame.step) | ISO 2768 medium, material aluminium 6061 or 6081, 8x M3 and 4x M2.5 Threads | 1 | [https://xometry.eu](https://xometry.eu) |
| [Wheel](mechanical/step/reactionwheel.stp) |  ISO 2768 fine, material brass MS58, one surface to H5 tolerance | 2 | [https://xometry.eu](https://xometry.eu) |
| [Lid](mechanical/step/lid.stp) | ISO 2768 medium, material aluminium 6061 or 6081 | 2 | [https://xometry.eu](https://xometry.eu) |

## Motors
| Item | Description | Qty | Vendor |
| ---- | ---- | --- | --- |
| Motors | T-Motor MN4006 KV380 (see note below) | 2 (sold as 1 set) | [T-Motor](https://store.tmotor.com/product/mn4006-kv380-motor-antigravity-type.html) |
| Encoder magnet | N35SH D=6mm t=2.5mm | 2 | [DigiKey](https://www.digikey.de/de/products/detail/radial-magnets-inc/9049/6030786) | 
| O-Ring | 56mm x 2mm NBR90 | 2 (get some spare!) | Maybe [McMaster](https://www.mcmaster.com/1247N172/) |

**Note on the motors**: There is a new [T-Motor MN4006 KV380 **EVO**](https://store.tmotor.com/de/product/mn4006-EVO-kv380-motor-antigravity-type.html) version of the motor with 150C temperature rated magnets, which I highly recommend using in new builds, however the tolerances in the flywheel are slightly different. The existing flywheel inner diameter is too large, which can only partially compensated by epoxy.

## Custom PCBs
| Item | Description | Qty | Vendor |
| ---- | ---- | --- | --- |
| Power Distribution [schematics](electronics/schematic/power-distribution.pdf), [gerbers](electronics/gerbers/power-distribution/power-distribution-v1.kicad_pcb_gerber.zip) | 4 layer, 1.6mm, 3/3 mil spacing, 0.15mm min hole | 1 | [http://pcbway.com/](http://pcbway.com/) |
| Motor Controller [schematics](electronics/schematic/micro-motor.pdf), [gerbers](electronics/gerbers/micro-motor-wb/micro-motor-v2.2WB.kicad_pcb_gerber.zip) | 4 layer, 1.6mm, 3/3 mil spacing, 0.15mm min hole | 2 | [http://pcbway.com/](http://pcbway.com/) |
| Flex PCB IMU 1 [schematics](electronics/schematic/imu-flex.pdf), [gerbers](electronics/gerbers/imu-flex-1/imu-flex-1-v1.kicad_pcb_gerber.zip) | 2 layer, 0.1mm, [FR4 Stiffener on frontside](electronics/gerbers/imu-flex-1/imu-flex-1-v1-stiffener_on_front.png), [max. dimensions 52x73.5mm](electronics/gerbers/imu-flex-1/imu-flex-1-v1-dimensions.png) | 1 | [http://pcbway.com/](http://pcbway.com/) | 
| Flex PCB IMU 2 [schematics](electronics/schematic/imu-flex.pdf), [gerbers](electronics/gerbers/imu-flex-2/imu-flex-2-v1.kicad_pcb_gerber.zip) | 2 layer, 0.1mm, [FR4 Stiffener on backside](electronics/gerbers/imu-flex-2/imu-flex-2-v1-stiffener_on_back.png), [max. dimensions 59x67.158mm](electronics/gerbers/imu-flex-2/imu-flex-2-v1-dimensions.png) | 1 | [http://pcbway.com/](http://pcbway.com/) |
| Compute Carrier [schematics](electronics/schematic/compute-carrier.pdf), [gerbers](electronics/gerbers/computecarrier/computecarrier-v1.kicad_pcb_gerber.zip) | 2 layer, 1.6mm | 1 | [http://pcbway.com/](http://pcbway.com/) |
| Cell Protection [schematics](electronics/schematic/cell-protection.pdf), [gerbers](electronics/gerbers/cell-protection/cell-protection-v1.kicad_pcb_gerber.zip) | 2 layer, 1.6mm | 1 | [http://pcbway.com/](http://pcbway.com/) |
| Cell Adapter [gerbers](electronics/gerbers/cell-adapter/cell-adapter-v1.kicad_pcb_gerber.zip) | 2 layer, 1.6mm, 2x press fit holes! | 2 | [http://pcbway.com/](http://pcbway.com/) |
| 3-phase cable extension [gerbers](electronics/gerbers/motorcon/motorcon-v1.kicad_pcb_gerber.zip) | 2 layer, 1.6mm | 1 | [http://pcbway.com/](http://pcbway.com/) |
| Encoder [schematics](electronics/schematic/encoder.pdf), [gerbers](electronics/gerbers/encoder/encoder-v1.kicad_pcb_gerber.zip) | 2 layer, 1.6mm | 2 | [http://pcbway.com/](http://pcbway.com/) | 

## Battery Pack
| Item | Description | Qty | Vendor |
| ---- | ---- | --- | --- |
| LiPo battery cells | Tattu 450mAH 75C XT30 Long 61x16mm cells from 2S,3S or 4S packs, e.g., TA-75C-450-2S1P-L | 6 cells (e.g., from 3 2S packs) | [fpv24.com](https://www.fpv24.com/de/tattu/tattu-batterie-lipo-akku-2s-450-mah-30c-xt30-lang) |
| Shrink wrap | 36mm wide when flat LiPO battery shrink wrap | <0.30m per robot | [Aliexpress](https://www.aliexpress.com/item/1005001749492020.html) |
| 3M ET 1339 tape | fiber reinforced polyester film tape, 15mm wide (can be more) | <1m per robot | [DigiKey](https://www.digikey.com/en/products/detail/3m-tc/1339-1-60/7571874) |
| Electrical grade silicone | Chip Quik EGS10C­20G | 1 | [DigiKey](https://www.digikey.com/en/products/detail/chip-quik-inc/EGS10C-20G/10059587) |
| Isolated NTC 10k | NRMR104F3435B2F NTC 10k B 3435K isolated leads | 2 | [DigiKey](https://www.digikey.com/en/products/detail/eaton-electronics-division/NRMR104F3435B2F/15927881) |

## Bolts and Fasteners
| Item | Description | Qty | Vendor |
| ---- | ---- | --- | --- |
| M2.5 nuts flat | DIN 439 M2.5 hex nut flat stainless A1/A2 | 8 | [McMaster](https://www.mcmaster.com/90710A025/) |
| M2.5x10mm set screw | DIN 913 set screw stainless A1/A2 M2.5x10 | 4 | [McMaster](https://www.mcmaster.com/92605A073/) |
| M2x6mm flat head screw | DIN 965 flat head torx stainless A1/A1 M2x6mm | 2 | [McMaster](https://www.mcmaster.com/92703A145/) |
| M2.5x6mm flat head screw | DIN 965 flat head torx stainless A1/A2 M2.5x6mm | 2 | [McMaster](https://www.mcmaster.com/92703A158/) |
| M3x20mm flat head screw | DIN 965 flat head torx stainless A1/A2 M3x20mm | 2 | [McMaster](https://www.mcmaster.com/92703A455/) |
| M2x4mm socket head screw | DIN912 socket head hex stainless A1/A2 M2x4mm | 6 | [McMaster](https://www.mcmaster.com/91292A004/) |
| M2x8mm socket head screw | DIN912 socket head hex stainless A1/A2 M2x8mm | 1 | [McMaster](https://www.mcmaster.com/91292A832/) |
| M2.5x4mm socket head screw | DIN912 socket head hex stainless A1/A2 M2.5x4mm | 5 | [McMaster](https://www.mcmaster.com/91292A015/) |
| M3x6mm socket head screw | DIN912 socket head hex stainless A1/A2 M3x6mm | 14 | [McMaster](https://www.mcmaster.com/91292A112/) |
| M3x8mm socket head screw | DIN912 socket head hex stainless A1/A2 M3x8mm | 2 | [McMaster](https://www.mcmaster.com/91292A111/) |
| M2x3mm threaded 3D print insert | Brass heat-set inserts for plastic, M2x3mm | 2 | [McMaster](https://www.mcmaster.com/94459A110/) | 
| M2.5x4mm threaded 3D print insert | Brass heat-set inserts for plastic, M2.5x4mm | 7 | [McMaster](https://www.mcmaster.com/94459A768/) | 

# Precrimped Cables and Connector Housings
| Item | Description | Qty | Vendor |
| ---- | ---- | --- | --- |
| JST-GH 2in | AGHGH28K51 precrimped 2in 28AWG single wire | 12 | [DigiKey](https://www.digikey.de/de/products/detail/jst-sales-america-inc/AGHGH28K51/6009448) |
| JST-GH 6in | AGHGH28K152 precrimped 6in 28AWG single wire | 6 | [DigiKey](https://www.digikey.de/de/products/detail/jst-sales-america-inc/AGHGH28K152/6009449) |
| JST-XH 2in | ASXHSXH22K51 precrimped 2in 22AWG single wire | 6 | [DigiKey](https://www.digikey.de/de/products/detail/jst-sales-america-inc/ASXHSXH22K51/6684930) |
| JST-XH 4in | ASXHSXH22K102 precrimped 4in 22AWG single wire | 3 | [DigiKey](https://www.digikey.de/de/products/detail/jst-sales-america-inc/ASXHSXH22K102/9961917) | 
| JST GH 6 pin plug housing | GHR-06V-S | 6 | [DigiKey](https://www.digikey.de/de/products/detail/jst-sales-america-inc/GHR-06V-S/807818) |
| JST XH 3 pin plug housing | XHP-3 | 3 | [DigiKey](https://www.digikey.de/de/products/detail/jst-sales-america-inc/XHP-3/1651017) |

## MISC
| Item | Description | Qty | Vendor |
| ---- | ---- | --- | --- |
| Quick set epoxy | Any 5 min Epoxy | a few drops | [McMaster](https://www.mcmaster.com/7541A76/) |
| Shielding copper tape | 3M Copper Tape | a small strip | [DigiKey](https://www.digikey.de/de/products/detail/3m-tc/3M-9876-15-12-X-12-SHEET/9841909) |
| Raspberry Pi CM4 | at least 8GB RAM, 16GB EMMC, with WIFI | 1 | [DigiKey](https://www.digikey.de/en/products/detail/raspberry-pi/SC0677/13530927) |
| Heat shrink | 3:1 heat shrink with glue for 22AWG single wires, 30-40mm length | 9 | |

## 3D Prints
| Item | Description | Qty | Vendor |
| ---- | ---- | --- | --- |
| [Battery housing](mechanical/meshes/battery_front.stl) | PETG or other high-temp filament, NO PLA | 1 | |
| [Battery lid](mechanical/meshes/battery_back.stl) | PETG or other high-temp filament, NO PLA | 1 | |
| [Internal frame 1](mechanical/step/electronics_frame_split_1.stp), can be printed in two pieces [a](mechanical/meshes/electronics_frame_split_1-a.stl) and [b](mechanical/meshes/electronics_frame_split_1-b.stl) and glued | PETG or other high-temp filament, NO PLA | 1 | |
| [Internal frame 2](mechanical/step/electronics_frame_split_2.stp), can be printed in two pieces [a](mechanical/meshes/electronics_frame_split_2-a.stl) and [b](mechanical/meshes/electronics_frame_split_2-b.stl) and glued | PETG or other high-temp filament, NO PLA | 1 | |
| Magnet holder | PETG or other high-temp filament, NO PLA | 1 | |
