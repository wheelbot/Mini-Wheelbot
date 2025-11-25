# Motor Calibration Tutorial

This guide explains how the in `calibrateMotors.py` script in `/wheelbot-lib/scripts/micro-motor/` operates and what steps are performed during motor calibration. Follow these steps if you want to manually calibrate the motors.

---

## Reference: Motor Calibration Command Meanings

| Command | Example Frame | Description |
|---------|----------------|--------------|
| `040#02` | `040#02` | Enable motor controller power |
| `040#01` | `040#01` | Disable motor controller power |
| `92` | `021#92`, `022#92` | **Disable cogging compensation** (required before recalibration) |
| `11` | `021#11`, `022#11` | Start encoder alignment routine |
| `12` | `021#12`, `022#12` | Start cogging compensation calibration |
| `46` | `021#46`, `022#46` | **Request special action status** (poll progress) |
| `91` | `021#91`, `022#91` | **Enable cogging compensation** after calibration |
| `23` | `021#23`, `022#23` | **Enable auto-reload** of cogging table on boot |
| `21` | `021#21`, `022#21` | **Save configuration to flash** |

## Setup

The can interface on the Mini Wheelbots CM4 is called motorCan.
Please first set the motor controller IDs via the given script.
```
setMotorControllerIDs.py
```

## 1. Connection to the Mini Wheelbot

For calibration, connect to the Mini Wheelbots CM4 via

```
ssh root@<robot-ip>
```


## 2. Enable Motor Controller Power

Calibration begins by powering the motor controllers:

```
cansend motorCan 040#02
```


---

## 3. Verify Motor Reachability

Send a CAN SYNC frame:

```
cansend motorCan 000#
```


Both motor controllers must respond with arbitration IDs:

```
081#
082#
```

Check this in a separate terminal on the CM4 via 

```
candump motorCan
```

If fewer than two replies appear, stop — check wiring, power, and CAN interface.

---

## 4. Disable Existing Cogging Compensation

Before recalibrating, previously stored cogging compensation must be turned off:

```
cansend motorCan 021#92
cansend motorCan 022#92
```

(`92` = disable cogging compensation)

---

## 5. Start Encoder Alignment

This routine aligns electrical and mechanical rotor positions:

```
cansend motorCan 021#11
cansend motorCan 022#11
```

Now wait **12 seconds** — the script does this automatically.

---

## 6. Start Cogging Compensation Learning

This step measures cogging torque across the motor revolution:

```
cansend motorCan 021#12
cansend motorCan 022#12
```

Motors will move — ensure no obstacles or load are attached.

---

## 7. Monitor Calibration Progress

Periodically request calibration status:

```
cansend motorCan 021#46
cansend motorCan 022#46
```

Motors respond with:

```
091#01 XX YY
092#01 XX YY
```


Interpret `XX`:

| XX | Meaning |
|----|---------|
| 06 | Calibration running |
| 07 | Calibration completed successfully |
| 08 | Calibration failed → stop |
| 09 | Calibration aborted → stop |

The script loops until **both motors return `07`**.

---

## 8. Apply and Store Calibration Results

After successful calibration:

```
cansend motorCan 021#91
cansend motorCan 022#91
```

Enables cogging compensation.

```
cansend motorCan 021#23
cansend motorCan 022#23
```

Enables auto-reload on startup.

```
cansend motorCan 021#21
cansend motorCan 022#21
```

Saves configuration to flash memory.

---

## 9. Disable Motor Controller Power

Calibration finishes by turning motors off:

```
cansend motorCan 040#01
```


This ensures safe shutdown.