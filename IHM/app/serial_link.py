import serial
import json
import time

from utils import db
from config import SERIAL_PORT, BAUD_RATE

def extract_json_payload(line):
    start = line.find("{")
    end = line.rfind("}")

    if start == -1 or end == -1 or end < start:
        return None

    candidate = line[start:end + 1]

    try:
        return json.loads(candidate)
    except json.JSONDecodeError:
        return None


def connect():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...")
        return ser
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return None


def read(ser, collection):
    line = ser.readline().decode("utf-8", errors="replace").strip()
    if line:
        payload = extract_json_payload(line)

        if payload is None:
            print(f"Ignored non-JSON serial line: {line}")
            return

        origin = payload.get("origin", "boat")
        destination = "boat" if origin == "server" else "server"

        db.push(
            origin=origin,
            destination=destination,
            data=json.dumps(payload, separators=(",", ":")),
            status="received",
            collection=collection
        )


# Le lien LoRa est half-duplex : un envoi unique entre souvent en collision
# avec le heartbeat du bateau (TX bloquant ~450 ms toutes les secondes) et se
# perd. On répète donc chaque commande quelques fois, espacées, pour qu'au
# moins une tombe dans une fenêtre d'écoute. Toutes les commandes du protocole
# sont idempotentes, donc la répétition est sans danger.
COMMAND_BURST_COUNT = 3
COMMAND_BURST_GAP_S = 0.35


def send(ser, collection):
    message = db.get_pending_message(collection)
    if message:
        data = json.dumps(message["data"], separators=(",", ":")) + "\n"
        payload = data.encode("utf-8")
        for i in range(COMMAND_BURST_COUNT):
            ser.write(payload)
            print(f"🚀 Envoi vers la carte ({i + 1}/{COMMAND_BURST_COUNT}) : {data.strip()}")
            time.sleep(COMMAND_BURST_GAP_S)



def main():
    collection = db.sync_client()
    ser = connect()
    if ser:   
        while True:
            try:
                read(ser, collection)
                send(ser, collection)

            except KeyboardInterrupt:
                # Handle Ctrl+C to exit
                print("Exiting...")
                ser.close()
                break

            except Exception as e:
                # Restart in case of failure (e.g. invalid char received)
                print(f"Error: {e}")
                print("Restarting...")
                pass


if __name__ == "__main__":
    main()
