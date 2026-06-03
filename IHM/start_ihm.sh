#!/bin/bash
# Démarre la pile complète AutoBoat IHM : MongoDB, serial_link.py, webserver.py, navigateur.
#
# Usage :
#   ./start_ihm.sh                  — utilise le port défini dans .env
#   ./start_ihm.sh /dev/ttyUSB0    — force le port série (prioritaire sur .env)
#   ./start_ihm.sh /dev/ttyUSB1    — idem avec un autre port

set -euo pipefail

IHM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_PYTHON="$IHM_DIR/.venv/bin/python"
COMPOSE_FILE="$IHM_DIR/docker-compose.yml"

# ── Couleurs ──────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "${GREEN}[OK]${NC}  $*"; }
info() { echo -e "${YELLOW}[..]${NC}  $*"; }
err()  { echo -e "${RED}[ERR]${NC} $*"; }

echo ""
echo "════════════════════════════════════════"
echo "   AutoBoat IHM — démarrage"
echo "════════════════════════════════════════"

# ── Port série ────────────────────────────────────────────────────────────────
# Un argument CLI remplace SERIAL_PORT_REAL depuis .env.
# python-dotenv ne remplace pas les variables déjà exportées dans le shell,
# donc les sous-processus héritent de la valeur forcée ici.
if [[ $# -ge 1 ]]; then
    export SERIAL_PORT_REAL="$1"
    export SIMULATION="false"
    ok "Port série (argument CLI) : $SERIAL_PORT_REAL"
else
    PORT_ENV=$(grep -oP '(?<=SERIAL_PORT_REAL=)\S+' "$IHM_DIR/.env" 2>/dev/null || echo "/dev/ttyUSB0")
    info "Port série (depuis .env) : $PORT_ENV"
    info "Pour changer : $0 /dev/ttyUSBx"
fi

# ── Vérification du venv ──────────────────────────────────────────────────────
if [[ ! -x "$VENV_PYTHON" ]]; then
    err "Environnement virtuel introuvable : $VENV_PYTHON"
    err "Créer avec : cd $IHM_DIR && python3 -m venv .venv && .venv/bin/pip install -r requirements.txt"
    exit 1
fi

# ── 1. MongoDB ────────────────────────────────────────────────────────────────
info "Démarrage MongoDB (Docker)..."
docker compose -f "$COMPOSE_FILE" up -d 2>&1 | grep -E "Started|Running|Created|error" || true

for i in $(seq 1 20); do
    "$VENV_PYTHON" -c \
        "import socket; socket.create_connection(('localhost', 27017), 1)" \
        2>/dev/null && break
    sleep 1
    [[ $i -eq 20 ]] && { err "MongoDB n'a pas démarré. Vérifier : docker compose -f $COMPOSE_FILE logs"; exit 1; }
done
ok "MongoDB prêt sur :27017"

# ── 2. serial_link.py ─────────────────────────────────────────────────────────
info "Arrêt de l'instance précédente serial_link.py..."
pkill -f "serial_link.py" 2>/dev/null && sleep 0.8 || true

info "Démarrage serial_link.py..."
cd "$IHM_DIR"
nohup "$VENV_PYTHON" app/serial_link.py \
    >> /tmp/autoboat_serial.log 2>&1 &
SERIAL_PID=$!
sleep 1

if kill -0 "$SERIAL_PID" 2>/dev/null; then
    ok "serial_link.py  PID $SERIAL_PID  →  /tmp/autoboat_serial.log"
else
    err "serial_link.py a planté au démarrage. Consulter /tmp/autoboat_serial.log"
    tail -10 /tmp/autoboat_serial.log
    exit 1
fi

# ── 3. webserver.py ───────────────────────────────────────────────────────────
info "Arrêt de l'instance précédente webserver.py..."
pkill -f "webserver.py" 2>/dev/null && sleep 0.8 || true

info "Démarrage webserver.py..."
nohup "$VENV_PYTHON" app/webserver.py \
    >> /tmp/autoboat_webserver.log 2>&1 &
SERVER_PID=$!

for i in $(seq 1 15); do
    "$VENV_PYTHON" -c \
        "import urllib.request; urllib.request.urlopen('http://localhost:5000', timeout=1)" \
        2>/dev/null && break
    sleep 1
    [[ $i -eq 15 ]] && {
        err "Le serveur web n'a pas répondu. Consulter /tmp/autoboat_webserver.log"
        tail -10 /tmp/autoboat_webserver.log
        exit 1
    }
done
ok "Serveur web  PID $SERVER_PID  →  http://localhost:5000"

# ── 4. Navigateur ────────────────────────────────────────────────────────────
info "Ouverture du navigateur..."
xdg-open http://localhost:5000 2>/dev/null \
    || open   http://localhost:5000 2>/dev/null \
    || err "Impossible d'ouvrir automatiquement — naviguer vers http://localhost:5000"

echo ""
echo "════════════════════════════════════════"
ok "IHM opérationnelle"
echo "   Navigateur  →  http://localhost:5000"
echo "   Logs :"
echo "     tail -f /tmp/autoboat_serial.log"
echo "     tail -f /tmp/autoboat_webserver.log"
echo ""
echo "   Pour arrêter :"
echo "     ./stop_ihm.sh"
echo "════════════════════════════════════════"
