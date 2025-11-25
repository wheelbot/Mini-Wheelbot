import os
import can
import time

bus = can.interface.Bus(channel='motorCan', bustype='socketcan')

responseError = False

try:
    for new_board_id in [0x01, 0x02]:
        print(f"\n[INFO] === Starting assignment for board {new_board_id} ===")

        # -----------------------------------------
        # First disable logic power, then enable for each board individually
        # -----------------------------------------
        print("[INFO] Disabling logic power...")
        msg_power = can.Message(arbitration_id=0x040, data=[0x03, 0x00, 0x00, 0x00], is_extended_id=False)
        bus.send(msg_power)
        time.sleep(1)
        print(f"[INFO] Enabling logic power for board {new_board_id}")
        if new_board_id == 0x01:
            msg_power = can.Message(arbitration_id=0x040, data=[0x03, 0x00, 0x00, 0x01], is_extended_id=False)
            bus.send(msg_power)
        if new_board_id == 0x02:
            msg_power = can.Message(arbitration_id=0x040, data=[0x03, 0x00, 0x01, 0x00], is_extended_id=False)
            bus.send(msg_power)
        time.sleep(4)

        # Send SYNC to make current board(s) announce themselves
        sync_msg = can.Message(arbitration_id=0x000, data=[], is_extended_id=False)
        bus.send(sync_msg)
        print("[INFO] Sent SYNC (000#). Waiting for response...")

        # Wait for response
        board_id = None
        start_time = time.time()
        timeout = 8
        response_msg = None

        while time.time() - start_time < timeout:
            msg = bus.recv(0.5)
            if msg is None:
                continue

            response_msg = msg
            print(f"[INFO] Board responded on ID 0x{msg.arbitration_id:03X}")

            # Board responds with arbitration_id = 0x080 + boardID (boardID can be 0x00–0x0FF)
            if msg.arbitration_id >= 0x080:
                board_id = msg.arbitration_id - 0x080
                response_msg = msg
                print(f"[INFO] Board responded on ID 0x{msg.arbitration_id:03X} → Board ID = {board_id}")
                break

        if board_id is None:
            print(f"[ERROR] No board response detected from board {new_board_id}.")
            responseError = True
            break

        # Request UUID from this board
        print("[INFO] Requesting UUID...")
        msg_request_uuid = can.Message(arbitration_id=0x020 + board_id, data=[0x43], is_extended_id=False)
        bus.send(msg_request_uuid)

        uuid = None
        start_time = time.time()
        timeout = 5

        while time.time() - start_time < timeout:
            msg = bus.recv(1.0)
            if not msg:
                continue
            if len(msg.data) >= 5 and msg.data[0] == 0x01:
                uuid_bytes = msg.data[1:5]
                uuid = "".join(f"{b:02X}" for b in uuid_bytes)
                print(f"[INFO] Received UUID: {uuid}")
                break

        if not uuid:
            print("[ERROR] Timeout waiting for UUID!")
            continue

        # Set new board ID using UUID
        data = [0x00] + list(bytes.fromhex(uuid)) + [0x00] + [new_board_id]
        msg_set_id = can.Message(arbitration_id=0x020 + board_id, data=data, is_extended_id=False)
        print(f"[INFO] Setting board ID → {new_board_id} (UUID {uuid})")
        bus.send(msg_set_id)
        time.sleep(5)
        msg_save_flash = can.Message(arbitration_id=0x020 + new_board_id, data=[0x021], is_extended_id=False)
        print("[INFO] Saving board ID to flash.")
        bus.send(msg_save_flash)
        time.sleep(5)
        print(f"[INFO] Board {new_board_id} assigned successfully!")

    if responseError:
        print("\n[Error] Error during assignment of IDs, at least one board did not respond to the sync message.")
    else:
        print("\n[INFO] All board IDs configured.")
        
    time.sleep(3)

    print("[INFO] Disabling logic power...")
    msg_power = can.Message(arbitration_id=0x040, data=[0x03, 0x00, 0x00, 0x00], is_extended_id=False)
    bus.send(msg_power)


finally:
    print("[INFO] Script has finished.")