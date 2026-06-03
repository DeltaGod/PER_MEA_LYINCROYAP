import os
from dotenv import load_dotenv
from pathlib import Path

load_dotenv()

SIMULATION = os.getenv("SIMULATION", "true").lower() == "true"

if SIMULATION:
    # Port pour socat sur Linux
    SERIAL_PORT = os.getenv("SERIAL_PORT_SIM", "/tmp/ttyV1") 
    DEFAULT_BAUD = 115200
    print("🛠️ Mode SIMULATION détecté (Port: /tmp/ttyV1)")
else:
    # Port pour la carte physique (USB)
    SERIAL_PORT = os.getenv("SERIAL_PORT_REAL", "/dev/ttyACM0")
    DEFAULT_BAUD = 115200
    print("🚢 Mode RÉEL détecté (Port: /dev/ttyACM0)")


MONGO_URI= os.getenv("MONGO_URI")

DB_NAME = os.getenv("DB_NAME")

BAUD_RATE = int(os.getenv("BAUD_RATE", DEFAULT_BAUD))

SERVER_PORT = int(os.getenv("SERVER_PORT", 5000))
