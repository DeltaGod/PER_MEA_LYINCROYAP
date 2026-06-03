///////////////////// Get elements///////////////////////////////////
let boatLatitudeDisplay = document.getElementById("boatLatitudeDisplay");
let boatLongitudeDisplay = document.getElementById("boatLongitudeDisplay");
let boatLastTimeDisplay = document.getElementById("boatLastTimeDisplay");
let routeFileInput = document.getElementById('routeFileInput');
let navigateButton = document.getElementById('navigateButton');
let boatHeadingDisplay = document.getElementById("boatHeadingDisplay");
let boatSailAngleDisplay = document.getElementById("boatSailAngleDisplay");
let boatRudderAngleDisplay = document.getElementById("boatRudderAngleDisplay");
let windObservationButton = document.getElementById('windObservationButton');
let windCommandButton = document.getElementById('windCommandButton');
let windDirectionDisplay = document.getElementById("windDirectionDisplay");
let waypointTotalDisplay = document.getElementById("waypointTotalDisplay");
let waypointCurrentDisplay = document.getElementById("waypointCurrentDisplay");
let boatModeDisplay = document.getElementById("boatModeDisplay");
let restartButton = document.getElementById('restartButton');
let startButton = document.getElementById('startButton');
let resetCommunicationsButton = document.getElementById('resetCommunicationsButton');

let boatMarker;     // The current boat location
let boatRoute;      // The route history for the boat
let pollingIntervalId = null;

// Etat de la batterie
let progressBattery = document.getElementById("progressBattery");
let percentBattery = document.getElementById("percentBattery");

// Puissance moteurs
let powerBar = document.getElementById("powerBar");
let powerValue = document.getElementById("powerValue");

// Vitesse du bateau
let prevLat = null;
let prevLon = null;
let prevTime = null;
let speed_kmh = document.getElementById('speed_kmh');
let speed_knots = document.getElementById('speed_knots');


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

// Polling the boat messages every seconds
function pollBoatMessages() {
    fetch("/api/messages").then(response => {
        if (!response.ok) {
            throw new Error('Network response was not ok');
        }
        return response.json(); // Parse JSON response
    }).then(data => {
        const message = data.message || {};
        //console.log('Data received:', data); // Handle the data
        
        if (data.hasOwnProperty('timestamp')) {
            updateBoatLastTime(data.timestamp);
        }
        if (Object.keys(message).length === 0) {
            return;
        }
        if (message.hasOwnProperty('mode')) {
            updateBoatMode(message.mode);
        }
        if (message.hasOwnProperty('location')) {
            updateBoatLocation(message.location, data.timestamp);
        }
        if (message.hasOwnProperty('servos')) {
            updateBoatAngles(message.servos.sail, message.servos.rudder);
        }
        if (message.hasOwnProperty('heading')) {
            updateBoatHeading(message.heading);
        }
        if (message.hasOwnProperty('wind')) {
            updateWindDirection(message.wind);
        }
        if (message.hasOwnProperty('waypoints')) {
            updateWaypoints(message.waypoints.total, message.waypoints.current);
        }
        if (message.hasOwnProperty('battery')) {
            updateBattery(message.battery);
        } else if (message.hasOwnProperty('bat')) {
            updateBatteryVoltage(message.bat);
        }
        if (message.hasOwnProperty('motor_power')) {
            updateMotorPower(message.motor_power.power, message.motor_power.percent);
        }
    }).catch(error => {
        console.error('There was a problem with the fetch operation:', error);
    });
}

function startMessagePolling() {
    if (pollingIntervalId !== null) {
        return;
    }

    pollBoatMessages();
    pollingIntervalId = setInterval(pollBoatMessages, 1000);
}

function stopMessagePolling() {
    if (pollingIntervalId === null) {
        return;
    }

    clearInterval(pollingIntervalId);
    pollingIntervalId = null;
}

startMessagePolling();


/////////////////////////////////////////////////////////////////////////////
// Define a function to update the boat location on the map, and compute the speed of the boat based on the previous location and time
/////////////////////////////////////////////////////////////////////////////

// Update the boat location
function updateBoatLocation(newLocation, timestamp) {
    const lat = newLocation[0];
    const lon = newLocation[1];
    const currentTime = new Date(timestamp).getTime() / 1000;

    if (prevLat !== null && prevLon !== null && prevTime !== null) {
        computeBoatSpeed(prevLat, prevLon, lat, lon, prevTime, currentTime);
    }

    prevLat = lat;
    prevLon = lon;
    prevTime = currentTime;

    const leafletMap = window.autoboatMap;

    if (!leafletMap || typeof leafletMap.addLayer !== "function") {
        console.error("Carte Leaflet indisponible dans script.js :", leafletMap);
        return;
    }

    const currentLatLng = [lat, lon];

    if (!boatMarker) {
        boatMarker = L.marker(currentLatLng).addTo(leafletMap);

        boatRoute = L.polyline([currentLatLng], {
            color: "blue",
            weight: 8,
            opacity: 0.5,
            clickable: false
        }).addTo(leafletMap);
    } else {
        boatMarker.setLatLng(currentLatLng);

        if (boatRoute) {
            boatRoute.addLatLng(currentLatLng);
        }
    }

    boatLatitudeDisplay.textContent = lat;
    boatLongitudeDisplay.textContent = lon;
}


function distanceGPS(lat1, lon1, lat2, lon2) {
    const R = 6371000; // rayon Terre en mètres

    const toRad = (deg) => deg * Math.PI / 180;

    const dLat = toRad(lat2 - lat1);
    const dLon = toRad(lon2 - lon1);

    const a =
        Math.sin(dLat / 2) ** 2 +
        Math.cos(toRad(lat1)) *
        Math.cos(toRad(lat2)) *
        Math.sin(dLon / 2) ** 2;

    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c; // distance en mètres
}

function computeBoatSpeed(lat1, lon1, lat2, lon2, t1, t2) {
    const distance = distanceGPS(lat1, lon1, lat2, lon2); 
    const dt = t2 - t1; // secondes

    // Sécurité critique pour éviter le NaN
    if (dt <= 0 || isNaN(dt)) {
        return 0;
    }

    if (dt <= 0) return 0;

    speed_kmh.textContent=((distance/dt) * 3.6).toFixed(2); // Affichage avec arrondis (toFixed(2)) pour que ce soit lisible
    speed_knots.textContent=((distance/dt) * 1.94384).toFixed(2);
}
    


/////////////////////////////////////////////////////////////////////////////
// Functions to update the boat information on the dashboard
/////////////////////////////////////////////////////////////////////////////

// Update the boat last time online
function updateBoatLastTime(newTime) {
    boatLastTimeDisplay.textContent = newTime;
}

// Update the boat heading
function updateBoatHeading(newHeading) {
    boatHeadingDisplay.textContent = newHeading;
}

// Update the boat angles (sail and rudder)
function updateBoatAngles(newSailAngle, newRudderAngle) {
    boatSailAngleDisplay.textContent = newSailAngle;
    boatRudderAngleDisplay.textContent = newRudderAngle;
}

// Update the wind direction
function updateWindDirection(newWindDirection) {
    windDirectionDisplay.textContent = newWindDirection;
}


// Update the waypoints details (total and current)
function updateWaypoints(newWaypointTotal, newWaypointCurrent) {
    waypointTotalDisplay.textContent = newWaypointTotal;
    waypointCurrentDisplay.textContent = newWaypointCurrent;
}

// Update the boat mode
function updateBoatMode(newBoatMode) {
    boatModeDisplay.textContent = newBoatMode;
}

/////////////////////////////////////////////////////////////////////////////
// Function to send the route waypoints to the boat, this is called after the user upload a new route
/////////////////////////////////////////////////////////////////////////////

async function sendRoute(waypoints) {
    console.log(waypoints);
    try {
        const response = await fetch('/api/send-route', {
            method: 'POST',
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({
                waypoints: waypoints
            })
        });

        const result = await response.json();

        if (!response.ok) {
            console.error("Erreur envoi route :", result);
            alert(result.message || "Erreur lors de l'envoi de la route.");
        }
    } catch (error) {
        console.error("Erreur réseau envoi route :", error);
        alert("Impossible de contacter le serveur.");
    }
}

/////////////////////////////////////////////////////////////////////////////
// Functions to update the battery and motor power information on the dashboard
/////////////////////////////////////////////////////////////////////////////

function updateBattery(percent)
{
    progressBattery.style.width = percent + "%";
    percentBattery.textContent = percent + "%";
}

function updateBatteryVoltage(volts)
{
    const voltage = Number(volts);
    if (Number.isNaN(voltage)) {
        return;
    }

    progressBattery.style.width = "0%";
    percentBattery.textContent = voltage.toFixed(2) + " V";
}

function updateMotorPower(power, percent)
{
    powerBar.style.width = percent + "%";
    powerValue.textContent = power.toFixed(1) + " W";
}

function resetBoatDisplay()
{
    const leafletMap = window.autoboatMap;

    if (leafletMap && boatMarker) {
        leafletMap.removeLayer(boatMarker);
    }

    if (leafletMap && boatRoute) {
        leafletMap.removeLayer(boatRoute);
    }

    boatMarker = null;
    boatRoute = null;
    prevLat = null;
    prevLon = null;
    prevTime = null;

    boatLastTimeDisplay.textContent = "?";
    boatLatitudeDisplay.textContent = "?";
    boatLongitudeDisplay.textContent = "?";
    boatHeadingDisplay.textContent = "?";
    boatSailAngleDisplay.textContent = "?";
    boatRudderAngleDisplay.textContent = "?";
    windDirectionDisplay.textContent = "?";
    waypointTotalDisplay.textContent = "?";
    waypointCurrentDisplay.textContent = "?";
    boatModeDisplay.textContent = "not-connected";
    speed_kmh.textContent = "0.00";
    speed_knots.textContent = "0.00";

    updateBattery(0);
    updateMotorPower(0, 0);
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

routeFileInput.addEventListener('change', (event) => {
    const file = event.target.files[0];

    if (file && file.type === "application/geo+json") {  // Ensure it's a GeoJSON file
        const reader = new FileReader();

        reader.onload = function (e) {
            const fileContent = e.target.result;
            // console.log("GeoJSON File Content:");
            // console.log(fileContent);
            let routeGeoJSON = parseGeoJSONRoute(fileContent);
            if (!routeGeoJSON) {
                return;
            }

            let waypoints = addRouteToMap(routeGeoJSON);
            sendRoute(waypoints);
        };

        reader.onerror = function () {
            alert("Error reading file");
        };

        // Read the file as text (GeoJSON is a JSON format, so it's read as text)
        reader.readAsText(file);
    } else {
        alert("Please upload a valid .geojson file");
    }
});

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

// Send the navigation order to the boat
navigateButton.addEventListener("click", () => {
    fetch('/api/navigate', {
        method: 'GET',
    });
});

// Send the wind observation order to the boat
windObservationButton.addEventListener("click", () => {
    fetch('/api/wind-observation', {
        method: 'GET',
    });
});

restartButton.addEventListener("click", () => {
    if (confirm("⚠️ Redémarrer ? ⚠️")) {
        fetch('/api/restart', {
            method: 'GET',
        });
    }
});

startButton.addEventListener("click", async () => {
    if (confirm("✅  Démarrer ? ✅ ")) {
        try {
            const response = await fetch('/api/start', {
                method: 'POST',
            });

            const result = await response.json();

            if (!response.ok) {
                console.error("Erreur start :", result);
                alert("Erreur lors du démarrage.");
                return;
            }

            startMessagePolling();
            console.log("Start OK :", result);
            alert("Système démarré.");

        } catch (error) {
            console.error("Erreur réseau start :", error);
            alert("Impossible de démarrer le système.");
        }
    }
});

resetCommunicationsButton.addEventListener("click", async () => {
    if (confirm("Réinitialiser toutes les communications ?")) {
        try {
            const response = await fetch('/api/reset-communications', {
                method: 'POST',
            });

            const result = await response.json();

            if (!response.ok) {
                console.error("Erreur reset communications :", result);
                alert("Erreur lors de la réinitialisation des communications.");
                return;
            }

            stopMessagePolling();
            resetBoatDisplay();
            console.log("Communications réinitialisées :", result);
            alert("Communications réinitialisées.");

        } catch (error) {
            console.error("Erreur réseau reset communications :", error);
            alert("Impossible de réinitialiser les communications.");
        }
    }
});

windCommandButton.addEventListener("click", () => {
    const direction = prompt("Direction du vent (en degrés) :")
    if (direction != null) {
        fetch('/api/wind-command/' + direction, {
            method: 'GET',
        });

    }
});

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

// Function to parse GeoJSON route and add it to the map if it's a LineString geometry type
function parseGeoJSONRoute(GeoJSONString) {
    try {
    let routeGeoJSON = JSON.parse(GeoJSONString);

    // Vérifier que c'est une FeatureCollection avec au moins une feature
        if (routeGeoJSON.type !== "FeatureCollection" || routeGeoJSON.features.length === 0) {
            alert("Please upload a valid .geojson file");
            return null;
        }

        // Vérifier que la première feature est un LineString
        if (routeGeoJSON.features[0].geometry.type !== "LineString") {
            alert("Please upload a valid .geojson file");
            return null;
        }

    return routeGeoJSON;
    } catch (error) {
    console.error("Error parsing GeoJSON:", error);
    alert("Please upload a valid .geojson file");
    return null;
    }
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

// Add the geojson LineString route to the map, this is the theoretical route of the boat
function addRouteToMap(routeGeoJSON) {
    const coordinates = routeGeoJSON.features[0].geometry.coordinates;
    const waypoints = coordinates.map((waypoint) => ({
        lat: waypoint[1],
        lon: waypoint[0],
        radius_m: 5.0
    }));

    if (typeof clearRoute === "function" && typeof addWaypoint === "function") {
        clearRoute();
        waypoints.forEach((waypoint) => {
            addWaypoint(waypoint.lat, waypoint.lon, waypoint.radius_m);
        });
    }

    return waypoints;
}
