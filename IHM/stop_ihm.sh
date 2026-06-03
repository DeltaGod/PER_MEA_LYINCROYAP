#!/bin/bash
# Arrête tous les processus de l'IHM AutoBoat : serial_link.py, webserver.py, MongoDB.

IHM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$IHM_DIR/docker-compose.yml"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "${GREEN}[OK]${NC}  $*"; }
info() { echo -e "${YELLOW}[..]${NC}  $*"; }

echo ""
echo "════════════════════════════════════════"
echo "   AutoBoat IHM — arrêt"
echo "════════════════════════════════════════"

info "Arrêt serial_link.py..."
pkill -f "serial_link.py" 2>/dev/null && ok "serial_link.py arrêté" || ok "serial_link.py n'était pas en cours"

info "Arrêt webserver.py..."
pkill -f "webserver.py" 2>/dev/null && ok "webserver.py arrêté" || ok "webserver.py n'était pas en cours"

info "Arrêt MongoDB (Docker)..."
docker compose -f "$COMPOSE_FILE" stop 2>&1 | grep -E "Stopped|error" || true
ok "MongoDB arrêté"

echo "════════════════════════════════════════"
ok "IHM arrêtée"
echo "   Relancer avec : ./start_ihm.sh [/dev/ttyUSBx]"
echo "════════════════════════════════════════"
echo ""
