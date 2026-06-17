# IHM Desktop — AutoBoat

IHM de escritorio en Python que reemplaza a la IHM web (`../IHM/`) **sin
navegador, sin servidor FastAPI, sin MongoDB y sin Docker**. Un solo proceso
habla directo con el transceiver por USB.

> La IHM web original sigue intacta en `../IHM/`. Esta es una alternativa
> independiente.

## Lanzar

```bash
./run.sh            # crea el venv la primera vez e inicia la app
```

o manualmente:

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python ihm_desktop.py
```

## Qué hace (paridad con la IHM web)

- **Mapa OSM** (tkintermapview) centrado en Brest, marcador del barco, traza
  recorrida, ruta planificada, predicción de trayectoria.
- **Telemetría en vivo**: modo, batería (V), velocidad (km/h y nudos), cap,
  servos (vela/safran), viento, coordenadas, waypoints total/actual, hora del
  último mensaje.
- **Panel de diagnóstico** (semáforos): Liaison (frescura + RSSI), GPS
  (fix/sat/hdop), Batterie (umbrales 2S), Radio (RC), Contrôle (auto/radio).
  Avisa cuando la liaison se pierde.
- **Brújula de viento** (la flecha apunta a la fuente del viento).
- **Medición de viento**: barra de progreso visible mientras llega `wobs`,
  con botón de cancelar.
- **Waypoints**: clic en el mapa para añadir, lista lateral, borrar selección,
  enviar ruta, borrar ruta, importar `.geojson` (LineString).
- **Órdenes**: Naviguer, Stop, Mesure vent, Envoi vent, Redémarrer,
  Reconnecter/Reset transceiver (pulso DTR/RTS).

## Puerto serie

El selector de puerto reemplaza al toggle Sim/Réel de la web:
- `auto` → auto-detección del transceiver por nº de serie CP2104 (`01C00B54`).
- elegir un `/dev/ttyUSB*` concreto.
- `/tmp/ttyV1` → apuntar al simulador (si se ejecuta aparte).

«Appliquer» reconecta sobre el puerto elegido.

## Notas

- El mapa descarga tiles de OpenStreetMap; **necesita internet**. Para campo
  sin red, tkintermapview admite caché SQLite de tiles (pre-descarga de la zona)
  — no configurado por defecto.
- Las órdenes se envían en ráfaga 3× (anti-colisión LoRa half-duplex), igual
  que el `serial_link.py` original.
- La lógica de la predicción de trayectoria (`nav.py`) es un port 1:1 de
  `navigation.h` / `nav.js`.
