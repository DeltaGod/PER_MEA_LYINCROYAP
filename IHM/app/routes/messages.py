import json
import os
import signal
import subprocess
import time
import urllib.parse
from pathlib import Path
from typing import Any, List

from fastapi import APIRouter
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

from utils import db


# ============================================================
# Configuration
# ============================================================

BASE_DIR = str(Path(__file__).resolve().parents[2])
PYTHON_BIN = f"{BASE_DIR}/.venv/bin/python"
MAX_LORA_PACKET_BYTES = 255

router = APIRouter()

messages_collection = db.sync_client()

processes: dict[str, subprocess.Popen] = {}

COMMUNICATION_PROCESS_PATTERNS = {
    "socat": "socat -d -d PTY,link=/tmp/ttySimu",
    "simu_boat": "AutoBoat_Simulation/simu_boat",
    "serial_link": "app/serial_link.py",
}

VIRTUAL_SERIAL_LINKS = ("/tmp/ttySimu", "/tmp/ttyV1")


# ============================================================
# Modèles de données
# ============================================================

class Waypoint(BaseModel):
    lat: float
    lon: float
    radius_m: float = Field(default=5.0, gt=0)


class RouteRequest(BaseModel):
    waypoints: List[Waypoint]


# ============================================================
# Fonctions utilitaires
# ============================================================

def push_command_to_boat(message: Any) -> JSONResponse:
    """
    Envoie une commande au bateau via MongoDB.
    La commande est ensuite récupérée par serial_link.py ou le contrôleur bateau.
    """

    data = {
        "origin": "server",
        "type": "command",
        "message": message
    }
    compact_data = json.dumps(data, separators=(",", ":"))

    if len(compact_data.encode("utf-8")) > MAX_LORA_PACKET_BYTES:
        return JSONResponse(
            {
                "status": "error",
                "message": "Commande trop longue pour un paquet LoRa.",
                "max_bytes": MAX_LORA_PACKET_BYTES,
                "actual_bytes": len(compact_data.encode("utf-8"))
            },
            status_code=400
        )

    try:
        db.push(
            "server",
            "boat",
            compact_data,
            "pending",
            messages_collection
        )

        return JSONResponse({
            "status": "success",
            "message_sent": message
        })

    except Exception as e:
        return JSONResponse(
            {"status": "error", "error": str(e)},
            status_code=500
        )


def build_waypoints_message(waypoints: List[Waypoint]) -> dict:
    """
    Transforme une liste de waypoints en message compatible avec ton protocole actuel.

    Format produit :
    {
        "waypoints": {
            "number": 2,
            "points": "48.39010000,-4.48600000,48.39040000,-4.48560000"
        }
    }

    Le firmware PER actuel ignore le rayon par waypoint et attend strictement
    des couples lat,lon dans la chaîne `points`.
    """

    points_string = ",".join(
        f"{wp.lat:.8f},{wp.lon:.8f}"
        for wp in waypoints
    )

    return {
        "waypoints": {
            "number": len(waypoints),
            "points": points_string
        }
    }


def parse_legacy_waypoints(waypoints_string: str) -> List[Waypoint]:
    """
    Compatibilité avec ton ancien endpoint :
    GET /send-route/{waypoints_string}

    Accepte :
    [
        [48.3901, -4.4860, 5],
        [48.3904, -4.4856, 5]
    ]

    ou :
    [
        {"lat": 48.3901, "lon": -4.4860, "radius_m": 5}
    ]
    """

    decoded = urllib.parse.unquote(waypoints_string)
    raw_waypoints = json.loads(decoded)

    waypoints: List[Waypoint] = []

    for item in raw_waypoints:
        if isinstance(item, dict):
            waypoints.append(Waypoint(**item))

        elif isinstance(item, list) or isinstance(item, tuple):
            if len(item) < 2:
                raise ValueError("Chaque waypoint doit contenir au minimum lat et lon.")

            lat = float(item[0])
            lon = float(item[1])
            radius_m = float(item[2]) if len(item) >= 3 else 5.0

            waypoints.append(Waypoint(
                lat=lat,
                lon=lon,
                radius_m=radius_m
            ))

        else:
            raise ValueError("Format de waypoint invalide.")

    return waypoints


def decode_message_data(data: Any) -> dict:
    if isinstance(data, dict):
        return data

    if not isinstance(data, str):
        return {}

    try:
        parsed = json.loads(data)
        return parsed if isinstance(parsed, dict) else {}
    except json.JSONDecodeError:
        start = data.find("{")
        end = data.rfind("}")

        if start == -1 or end == -1 or end < start:
            return {}

        try:
            parsed = json.loads(data[start:end + 1])
            return parsed if isinstance(parsed, dict) else {}
        except json.JSONDecodeError:
            return {}


def is_running(name: str) -> bool:
    return name in processes and processes[name].poll() is None


def launch_process(name: str, command, shell: bool = False) -> dict:
    """
    Lance un processus si celui-ci n'est pas déjà actif.
    """

    if is_running(name):
        return {
            "name": name,
            "status": "already_running",
            "pid": processes[name].pid
        }

    try:
        process = subprocess.Popen(
            command,
            cwd=BASE_DIR,
            shell=shell,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True
        )

        processes[name] = process

        return {
            "name": name,
            "status": "started",
            "pid": process.pid
        }

    except Exception as e:
        return {
            "name": name,
            "status": "error",
            "error": str(e)
        }


def wait_for_pid_exit(pid: int, timeout: float = 2.0) -> bool:
    deadline = time.time() + timeout

    while time.time() < deadline:
        try:
            os.kill(pid, 0)
            time.sleep(0.1)
        except ProcessLookupError:
            return True
        except PermissionError:
            return False

    return False


def terminate_process_group(name: str, process: subprocess.Popen) -> dict:
    """
    Arrête un processus lancé par /start.
    start_new_session=True permet de tuer tout son groupe.
    """

    if process.poll() is not None:
        return {
            "name": name,
            "pid": process.pid,
            "status": "already_stopped",
            "return_code": process.returncode
        }

    try:
        os.killpg(process.pid, signal.SIGTERM)
        stopped = wait_for_pid_exit(process.pid)

        if not stopped:
            os.killpg(process.pid, signal.SIGKILL)
            stopped = wait_for_pid_exit(process.pid)

        return {
            "name": name,
            "pid": process.pid,
            "status": "stopped" if stopped else "stop_requested"
        }

    except ProcessLookupError:
        return {
            "name": name,
            "pid": process.pid,
            "status": "already_stopped"
        }

    except Exception as e:
        return {
            "name": name,
            "pid": process.pid,
            "status": "error",
            "error": str(e)
        }


def find_pids_by_pattern(pattern: str) -> List[int]:
    result = subprocess.run(
        ["pgrep", "-f", pattern],
        capture_output=True,
        text=True,
        check=False
    )

    if result.returncode not in (0, 1):
        raise RuntimeError(result.stderr.strip() or "pgrep a échoué.")

    pids = []

    for line in result.stdout.splitlines():
        try:
            pid = int(line.strip())
        except ValueError:
            continue

        if pid != os.getpid():
            pids.append(pid)

    return pids


def terminate_external_pid(name: str, pid: int, stopped_groups: set[int]) -> dict:
    try:
        pgid = os.getpgid(pid)

        if pgid in stopped_groups:
            return {
                "name": name,
                "pid": pid,
                "pgid": pgid,
                "status": "already_requested"
            }

        os.killpg(pgid, signal.SIGTERM)
        stopped_groups.add(pgid)
        stopped = wait_for_pid_exit(pid)

        if not stopped:
            os.killpg(pgid, signal.SIGKILL)
            stopped = wait_for_pid_exit(pid)

        return {
            "name": name,
            "pid": pid,
            "pgid": pgid,
            "status": "stopped" if stopped else "stop_requested"
        }

    except ProcessLookupError:
        return {
            "name": name,
            "pid": pid,
            "status": "already_stopped"
        }

    except Exception as e:
        return {
            "name": name,
            "pid": pid,
            "status": "error",
            "error": str(e)
        }


def remove_virtual_serial_links() -> list[dict]:
    results = []

    for path in VIRTUAL_SERIAL_LINKS:
        if not os.path.exists(path) and not os.path.islink(path):
            results.append({
                "path": path,
                "status": "missing"
            })
            continue

        try:
            os.unlink(path)
            results.append({
                "path": path,
                "status": "removed"
            })

        except Exception as e:
            results.append({
                "path": path,
                "status": "error",
                "error": str(e)
            })

    return results


# ============================================================
# API : récupération des messages bateau
# ============================================================

@router.get("/messages")
async def get_messages():
    """
    Récupère le dernier message envoyé par le bateau.
    """

    try:
        last_message = messages_collection.find_one(
            {"origin": "boat"},
            sort=[("timestamp", -1)]
        )

        if not last_message:
            return JSONResponse({
                "message": {},
                "timestamp": None
            })

        inner_data = decode_message_data(last_message.get("data", {}))

        boat_message = inner_data.get("message", {})
        timestamp = last_message.get("timestamp")

        return JSONResponse({
            "message": boat_message,
            "timestamp": timestamp.isoformat() if timestamp else None
        })

    except Exception as e:
        return JSONResponse(
            {"status": "error", "error": str(e)},
            status_code=500
        )


# ============================================================
# API : route / waypoints
# ============================================================

@router.post("/send-route")
def send_route(route: RouteRequest):
    """
    Endpoint propre pour envoyer une route au bateau.

    Reçoit :
    {
        "waypoints": [
            {"lat": 48.3901, "lon": -4.4860, "radius_m": 5},
            {"lat": 48.3904, "lon": -4.4856, "radius_m": 5}
        ]
    }
    """

    if len(route.waypoints) == 0:
        return JSONResponse(
            {
                "status": "error",
                "message": "Aucun waypoint reçu."
            },
            status_code=400
        )

    message = build_waypoints_message(route.waypoints)
    return push_command_to_boat(message)


@router.get("/send-route/{waypoints_string}")
def send_route_legacy(waypoints_string: str):
    """
    Ancien endpoint conservé pour compatibilité.
    À éviter pour la version finale.
    """

    try:
        waypoints = parse_legacy_waypoints(waypoints_string)

        if len(waypoints) == 0:
            return JSONResponse(
                {
                    "status": "error",
                    "message": "Aucun waypoint reçu."
                },
                status_code=400
            )

        message = build_waypoints_message(waypoints)
        return push_command_to_boat(message)

    except Exception as e:
        return JSONResponse(
            {
                "status": "error",
                "message": "Impossible de parser les waypoints.",
                "error": str(e)
            },
            status_code=400
        )


# ============================================================
# API : commandes bateau
# ============================================================

@router.get("/navigate")
def send_navigate():
    """
    Démarre la navigation autonome.
    """

    return push_command_to_boat({"navigate": True})


@router.get("/wind-observation")
def send_wind_observation():
    """
    Lance l'observation / mesure de direction du vent.
    """

    return push_command_to_boat({"wind-observation": True})


@router.get("/wind-command/{direction}")
def send_wind_command(direction: int):
    """
    Envoie une consigne de direction du vent.

    direction : angle en degrés.
    """

    message = {
        "wind-command": {
            "value": direction
        }
    }

    return push_command_to_boat(message)


@router.get("/restart")
def send_restart():
    """
    Redémarre le contrôleur embarqué du bateau.
    """

    return push_command_to_boat({"restart": True})


# ============================================================
# API : lancement simulation / liaison série
# ============================================================

@router.post("/start")
def send_start():
    """
    Lance :
    1. socat
    2. le simulateur bateau
    3. serial_link.py
    """

    results = []

    # 1. Lancer socat
    results.append(
        launch_process(
            "socat",
            [
                "socat",
                "-d",
                "-d",
                "PTY,link=/tmp/ttySimu,raw,echo=0",
                "PTY,link=/tmp/ttyV1,raw,echo=0"
            ]
        )
    )

    # 2. Attendre la création des ports virtuels
    for _ in range(30):
        if os.path.exists("/tmp/ttySimu") and os.path.exists("/tmp/ttyV1"):
            break

        time.sleep(0.1)

    else:
        return JSONResponse(
            {
                "status": "error",
                "message": "Les ports /tmp/ttySimu et /tmp/ttyV1 n'ont pas été créés.",
                "results": results
            },
            status_code=500
        )

    # 3. Lancer le simulateur bateau
    results.append(
        launch_process(
            "simu_boat",
            "./AutoBoat_Simulation/simu_boat > /tmp/ttySimu",
            shell=True
        )
    )

    # 4. Lancer serial_link.py avec le Python du venv
    results.append(
        launch_process(
            "serial_link",
            [
                PYTHON_BIN,
                "app/serial_link.py"
            ]
        )
    )

    return JSONResponse({
        "status": "ok",
        "message": "Commandes Start lancées.",
        "results": results
    })


@router.post("/reset-communications")
def reset_communications():
    """
    Arrête la liaison simulation/série et vide les messages de communication.
    """

    tracked_results = []

    for name, process in list(processes.items()):
        tracked_results.append(terminate_process_group(name, process))
        processes.pop(name, None)

    external_results = []
    stopped_groups: set[int] = set()

    for name, pattern in COMMUNICATION_PROCESS_PATTERNS.items():
        try:
            pids = find_pids_by_pattern(pattern)

            if not pids:
                external_results.append({
                    "name": name,
                    "pattern": pattern,
                    "status": "not_found"
                })
                continue

            for pid in pids:
                external_results.append(
                    terminate_external_pid(name, pid, stopped_groups)
                )

        except Exception as e:
            external_results.append({
                "name": name,
                "pattern": pattern,
                "status": "error",
                "error": str(e)
            })

    serial_link_results = remove_virtual_serial_links()
    delete_result = messages_collection.delete_many({})

    return JSONResponse({
        "status": "ok",
        "message": "Communications réinitialisées.",
        "tracked_processes": tracked_results,
        "external_processes": external_results,
        "serial_links": serial_link_results,
        "deleted_messages": delete_result.deleted_count
    })


@router.get("/process-status")
def get_process_status():
    """
    Permet de vérifier quels processus sont actifs.
    """

    status = {}

    for name, process in processes.items():
        status[name] = {
            "pid": process.pid,
            "running": process.poll() is None,
            "return_code": process.poll()
        }

    return JSONResponse({
        "status": "ok",
        "processes": status
    })
