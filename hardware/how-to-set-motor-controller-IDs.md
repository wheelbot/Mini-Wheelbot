# Motor Controller ID Assignment Tutorial

This guide explains how the `setMotorControllerIDs.py` script in `/wheelbot-lib/scripts/micro-motor/` operates and what steps are performed to assign CAN IDs to the motor controllers on the Mini Wheelbot.  
Follow these steps if you want to manually set the board IDs.

---

## Reference: ID Assignment Command Meanings

| Command | Example Frame | Description |
|---------|----------------|--------------|
| `040#01` | `040#01` | Disable drive and logic power to both boards |
| `040#02` | `040#02` | Enable drive and logic power to both boards |
| `040#03xxxx` | `040#03000000` | Disable drive and logic power to both boards |
| `040#03000001` | `040#03000001` | Enable logic power only for board 1 |
| `040#03000100` | `040#03000100` | Enable logic power only for board 2 |
| `040#03010000` | `040#03010000` | Enable drive power and disable logic power to both boards |
| `040#03010101` | `040#03010101` | Enable drive and logic power to both boards |
| `000#` | `000#` | SYNC — request boards to respond |
| `020+ID#43` | `021#43`, `022#43` | Request UUID from board |
| `020+ID#00<UUID>00<newID>` | `021#00xxxxxxxx0001` | Assign new CAN ID |
| `020+newID#21` | `021#21`, `022#21` | Save new board ID to flash |


---

This ReadMe gives can commands with arbitration IDs based on the final state, where motor controller 1/2 have ID=1/2. After initial flash of the motor controller, these IDs are not set yet. In this state, the current board ID can be retrieved by sending a Sync message (000#), the response arbitration ID is 0x80 + {current board ID}. 
For example, a response arbitration ID of 0x17F corresponds to 0x80+0xFF, so {current board ID}= 0xFF = 255. 
Then, all can commands that communicate to the motor controller are sent with arbitration ID 0x20 + {current board ID}.

## Setup

The CAN interface on the Mini Wheelbot CM4 is called `motorCan`.

Run ID assignment script from the CM4:

```
python3 setMotorControllerIDs.py
```


---

## 1. Connect to the Mini Wheelbot

```
ssh root@<robot-ip>
```


---

## 2. Disable Logic Power to Both Motor Controllers

Ensures no conflicting responses during ID assignment:

```
cansend motorCan 040#03000000
```

## 3. Enable Logic Power for only Board 1

This isolates the first controller:

```
cansend motorCan 040#03000001
```

## 4. Request Board response

Send SYNC:

```
cansend motorCan 000#
```

Expected response shown via candump motorCan (exemplary):

```
081#
```

➡︎ Means a board currently has ID 0x01. Response arbitration ID is 080 + board ID. 

If no response, check wiring and power.
## 5. Request UUID From the Active Board

```
cansend motorCan 021#43
```

Board replies with 5 data bytes, e.g.:

```
0x01 AA BB CC DD
```

Where AABBCCDD = unique UUID.
## 6. Assign New Board ID Using UUID

Example: Assign ID 0x01 to this board

```
cansend motorCan 021#00AABBCCDD0001
```

## 7. Save Assigned ID to Flash

```
cansend motorCan 021#21
```

Wait ~2 seconds — do not power-cycle during this time.
Now Board 1 successfully stored the new ID.

## 8. Repeat Steps 2–7 for Board 2

Enable only board 2 logic power:

```
cansend motorCan 040#03000100
```

Then perform:

```
cansend motorCan 000#
cansend motorCan 022#43
cansend motorCan 022#00<UUID>0002
cansend motorCan 022#21
```

## 9. Verify Both Boards Respond on CAN

Send SYNC again:

```
cansend motorCan 040#03000101
cansend motorCan 000#
```

Expected:

```
081#
082#
```

If yes → IDs are correctly assigned.
## 10. Disable Logic Power After Completion

```
cansend motorCan 040#01
```


This prevents accidental motor activation.
You may now proceed with motor calibration