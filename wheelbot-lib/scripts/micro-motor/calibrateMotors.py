import can
import time

bus = can.interface.Bus(channel='motorCan', bustype='socketcan')

def send_hex(arbitration_id, hex_str):
    """
    Convenience function to send CAN frames like "021#92".
    """
    data = bytes.fromhex(hex_str)
    msg = can.Message(arbitration_id=arbitration_id, data=list(data), is_extended_id=False)
    bus.send(msg)

def wait_for_boards(expected=2, timeout=5.0):
    """
    Send SYNC (000#) and wait until 'expected' boards respond.
    Returns a list of boardIDs (0x01, 0x02, ...)
    """
    print("[INFO] Sending SYNC...")
    bus.send(can.Message(arbitration_id=0x000, data=[], is_extended_id=False))

    found = []
    start = time.time()

    while time.time() - start < timeout:
        msg = bus.recv(0.5)
        if not msg:
            continue

        if msg.arbitration_id >= 0x080:
            board_id = msg.arbitration_id - 0x080
            if board_id not in found:
                found.append(board_id)
                print(f"[INFO] Detected board ID: {board_id}")

            if len(found) == expected:
                return found

    return None


def read_special_action_status(board_id, timeout=0.5):
    """
    Sends 0x46 diagnostic request to the motor
    Returns tuple (status, progress):
      status = XX
      progress = YY
    or None if timeout.
    """
    arb = 0x020 + board_id
    send_hex(arb, "46")  # 021#46 or 022#46
    start = time.time()

    while time.time() - start < timeout:
        msg = bus.recv(0.2)
        if not msg:
            continue

        # Expect 091#01 XX YY for board 1 → arb = 0x091
        # Expect 092#01 XX YY for board 2 → arb = 0x092
        if msg.arbitration_id == (0x090 + board_id):
            if len(msg.data) >= 3 and msg.data[0] == 0x01:
                xx = msg.data[1]
                yy = msg.data[2]
                return xx, yy

    return None


try:
    print("\n=== MOTOR CALIBRATION SCRIPT STARTED ===")

    # -------------------------------------
    # 1. Enable motor power
    # -------------------------------------
    print("[INFO] Enabling motor power (040#02)")
    send_hex(0x040, "02")
    time.sleep(4)

    # -------------------------------------
    # 2. Check if both motors are reachable
    # -------------------------------------
    boards = wait_for_boards(expected=2)
    if boards is None or len(boards) != 2:
        print("[ERROR] Motors not reachable. Aborting calibration.")
        raise SystemExit

    print(f"[INFO] Found both boards: {boards}")

    # -------------------------------------
    # 3. Start calibration sequence
    # -------------------------------------
    print("[INFO] Starting calibration...")

    # Step: send 021#92 and 022#92
    send_hex(0x021, "92")
    send_hex(0x022, "92")

    # Step: send 021#11 and 022#11
    send_hex(0x021, "11")
    send_hex(0x022, "11")

    print("[INFO] Calibrating encoders...")
    time.sleep(12)

    # Step: send 021#12 and 022#12
    send_hex(0x021, "12")
    time.sleep(1)
    send_hex(0x022, "12")

    # -------------------------------------
    # 4. Wait for calibration to finish
    # -------------------------------------
    print("[INFO] Calibrating cogging compensation...")
    print("[INFO] Waiting for cogging calibration to complete...")

    while True:
        status = []
        for board in boards:
            s = read_special_action_status(board)
            if s is None:
                print(f"[WARN] No status reply from board {board}. Retrying...")
                status.append((None, None))
                continue
            status.append(s)

        # Each status is (XX, YY)
        xx_values = [s[0] for s in status]

        # Print status meaning for each board
        for (board, (xx, yy)) in zip(boards, status):
            if xx is None:
                print(f"[WARN] Board {board}: No data")
                continue

            if xx == 0x06:
                print(f"[INFO] Board {board}: Status 06 (Calibration running)")
            elif xx == 0x07:
                print(f"[INFO] Board {board}: Status 07 (Calibration success)")
            elif xx == 0x08:
                print(f"[ERROR] Board {board}: Status 08 (Calibration failed)")
            elif xx == 0x09:
                print(f"[ERROR] Board {board}: Status 09 (Calibration aborted)")
            else:
                print(f"[INFO] Board {board}: Status {xx:02X} (Unknown)")

        # Abort if any board reports failure
        if any(x == 0x08 for x in xx_values):
            print("[ERROR] Calibrating the cogging compensation failed.")
            raise SystemExit

        # Abort if any board reports abort
        if any(x == 0x09 for x in xx_values):
            print("[ERROR] Calibration aborted by motor.")
            raise SystemExit

        # Success only if both are done
        if all(x == 0x07 for x in xx_values):
            print("[INFO] Calibration successful for all motors!")
            break

        # Otherwise still running
        print("[INFO] Calibration still running...")
        time.sleep(5)


    # -------------------------------------
    # 5. Final steps
    # -------------------------------------
    print("[INFO] Sending finalization commands...")
    print("[INFO] Enabling cogging compensation...")

    send_hex(0x021, "91")
    send_hex(0x022, "91")
    time.sleep(2)

    print("[INFO] Enabling auto reload...")
    send_hex(0x021, "23")
    send_hex(0x022, "23")
    time.sleep(3)

    print("[INFO] Saving to flash...")
    send_hex(0x021, "21")
    send_hex(0x022, "21")  

    time.sleep(5)

    # -------------------------------------
    # 6. Disable motor power
    # -------------------------------------
    print("[INFO] Disabling motor power (040#01)")
    send_hex(0x040, "01")

    print("\n=== MOTOR CALIBRATION COMPLETED SUCCESSFULLY ===")

finally:
    print("[INFO] Calibration script finished.")
