# Utiliser et créer des simulations

Ce guide explique comment compiler, lancer, modifier et créer un scénario. Il
est prévu pour une personne qui découvre entièrement le projet.

Pour comprendre le code interne, voir
[CODE_ARCHITECTURE.md](CODE_ARCHITECTURE.md). Pour comprendre les décisions du
bateau, voir [NAVIGATION.md](NAVIGATION.md).

## 1. Prérequis

Il faut :

- un terminal Linux ou macOS ;
- `make` ;
- un compilateur compatible C++17, par exemple `g++` ;
- un navigateur web ;
- une connexion Internet pour afficher Leaflet et le fond de carte.

Vérification rapide :

```bash
make --version
g++ --version
```

## 2. Compiler

Depuis la racine du dépôt :

```bash
cd simulation
make
```

La compilation crée l'exécutable :

```text
simulation/boat_simulator
```

Pour supprimer les fichiers générés puis recompiler :

```bash
make clean
make
```

## 3. Lancer

Toujours depuis le dossier `simulation` :

```bash
./boat_simulator
```

La commande exécute actuellement les sept scénarios définis dans
`sim_main.cpp`, puis crée :

```text
output/simulation.html
```

On peut également utiliser :

```bash
make run
```

### Limitation actuelle des arguments

Le programme ne lit pas encore les arguments de ligne de commande. Ainsi :

```bash
./boat_simulator 1
```

exécute tout de même les sept scénarios.

Pour la même raison, `make run-all` relance actuellement l'ensemble des
scénarios plusieurs fois. Il est donc préférable d'utiliser simplement
`./boat_simulator` ou `make run`.

## 4. Ouvrir le résultat

Ouvrir le fichier suivant dans un navigateur :

```text
simulation/output/simulation.html
```

La page possède un onglet par scénario. Elle permet de :

- voir les waypoints et la trajectoire ;
- lancer ou mettre en pause l'animation ;
- déplacer le curseur temporel ;
- lire le cap, la vitesse et les commandes ;
- identifier le mode de navigation.

Si la page s'ouvre mais que la carte reste vide, vérifier la connexion Internet.
La trajectoire est intégrée au fichier, mais Leaflet et les tuiles de carte sont
chargés depuis des services externes.

## 5. Comprendre un scénario existant

Les scénarios se trouvent dans `sim_main.cpp`. Chacun est une fonction qui
renvoie un `ScenarioData`.

Exemple minimal :

```cpp
ScenarioData runMyScenario() {
    SimTime::init();

    SimulatedBoat boat;
    boat.init(
        48.359045,  // latitude initiale
        -4.550422,  // longitude initiale
        215.0,      // direction d'où vient le vent
        8.0,        // vitesse du vent en m/s
        90.0        // cap initial du bateau
    );

    boat.addWaypoint(
        48.342791,  // latitude cible
        -4.531538   // longitude cible
    );

    boat.startWindObservation();
    boat.runSimulation(60 * 1000UL, 100);

    boat.setWind(215.0, 8.0);
    boat.startNavigation();
    boat.runSimulation(2 * 60 * 60 * 1000UL, 100);

    return {
        "Mon scénario",
        boat.getHistory(),
        boat.getWaypointPairs(),
        215.0,
        8.0
    };
}
```

## 6. Signification des paramètres

### Position GPS

Une position est donnée sous la forme :

```text
latitude, longitude
```

Exemple près de Brest :

```text
48.359045, -4.550422
```

Une longitude négative correspond ici à une position à l'ouest du méridien de
Greenwich.

### Angles

Les angles utilisent les degrés :

| Angle | Direction |
|---:|---|
| `0` ou `360` | nord |
| `90` | est |
| `180` | sud |
| `270` | ouest |

La direction du vent indique son origine. `215°` signifie que le vent vient du
sud-ouest.

### Durées

Les durées sont en millisecondes :

```cpp
30 * 1000UL                 // 30 secondes
10 * 60 * 1000UL            // 10 minutes
2 * 60 * 60 * 1000UL        // 2 heures
7 * 24 * 60 * 60 * 1000UL   // 7 jours
```

Le suffixe `UL` évite un dépassement de capacité pendant les multiplications.

### Pas de temps

Le deuxième argument de `runSimulation` est le pas de temps :

```cpp
boat.runSimulation(duree, 100);
```

Valeurs conseillées :

| Besoin | Pas indicatif |
|---|---:|
| mouvement détaillé | `50` à `100 ms` |
| trajet de plusieurs heures | `100` à `500 ms` |
| trajet de plusieurs jours | `1000 ms` |

Les inerties du bateau sont calculées en fonction du temps et restent donc
comparables entre ces valeurs. En revanche, la navigation prend une décision à
chaque pas : un grand pas rend ses corrections moins fréquentes.

Éviter un pas supérieur à `10000 ms` avec le code actuel : l'affichage de
progression suppose qu'au moins une itération existe par tranche de dix
secondes.

## 7. Modifier un scénario

Les changements les plus courants sont :

### Changer le départ

Modifier les deux premiers paramètres de `boat.init(...)`.

### Changer le vent

Modifier la direction et la vitesse dans `boat.init(...)`, puis conserver les
mêmes valeurs dans `boat.setWind(...)` et dans le `ScenarioData` retourné.

```cpp
boat.setWind(230.0, 7.5);
```

`setWind(...)` peut aussi être appelé entre deux périodes de simulation pour
simuler un changement de vent :

```cpp
boat.runSimulation(60 * 60 * 1000UL, 100);
boat.setWind(250.0, 9.0);
boat.runSimulation(60 * 60 * 1000UL, 100);
```

### Ajouter plusieurs waypoints

Appeler `addWaypoint(...)` dans l'ordre souhaité :

```cpp
boat.addWaypoint(48.342791, -4.531538);
boat.addWaypoint(48.321877, -4.472607);
boat.addWaypoint(48.304197, -4.476354);
```

Le bateau passe automatiquement au suivant quand le waypoint courant est
atteint. La distance d'arrivée est fixée à `10 m` par
`WAYPOINT_DISTANCE_SIM` dans `sim_boat.hpp`.

### Allonger la simulation

Augmenter le premier argument :

```cpp
boat.runSimulation(7 * 24 * 60 * 60 * 1000UL, 1000);
```

La simulation s'arrête avant la durée maximale si tous les waypoints ont déjà
été atteints.

## 8. Créer un nouveau scénario

1. Copier la structure d'une fonction `runScenario...()` dans `sim_main.cpp`.
2. Donner un nom unique à la nouvelle fonction.
3. Initialiser le bateau et ajouter les waypoints.
4. Choisir une durée maximale et un pas de temps.
5. Retourner un `ScenarioData`.
6. Ajouter l'appel dans `main()`.

Exemple d'ajout dans `main()` :

```cpp
std::vector<ScenarioData> scenarios;
scenarios.push_back(runScenario1());
scenarios.push_back(runMyScenario());
```

Il n'est pas nécessaire de modifier le `Makefile` si tout le scénario reste
dans `sim_main.cpp`.

Après chaque modification :

```bash
make
./boat_simulator
```

## 9. Choisir un bon scénario de test

Un scénario utile vérifie une seule idée principale. Par exemple :

- waypoint presque dans l'axe actuel ;
- waypoint directement face au vent ;
- waypoint directement sous le vent ;
- plusieurs waypoints rapprochés ;
- waypoint dépassé à cause d'un grand pas de temps ;
- changement de vent pendant le trajet ;
- trajet suffisamment long pour observer plusieurs changements de bord.

Pour diagnostiquer un problème, commencer avec un seul waypoint et un vent
constant. Ajouter ensuite les difficultés une par une.

## 10. Vérifier le résultat

Après le lancement, contrôler :

1. que le programme annonce la fin sans erreur ;
2. que `output/simulation.html` a été régénéré ;
3. que le bateau se dirige globalement vers le waypoint ;
4. que le mode affiché correspond à la situation ;
5. que le waypoint atteint devient le waypoint suivant ;
6. que les changements de cap restent progressifs ;
7. que la trajectoire reste cohérente après un changement de vent.

Une trajectoire en zigzag n'est pas forcément une erreur. Face au vent, un
voilier ne peut pas aller directement vers sa cible et doit tirer des bords.

## 11. Problèmes fréquents

### `make: command not found` ou `g++: command not found`

Installer les outils de compilation du système.

### La compilation utilise un ancien résultat

```bash
make clean
make
```

### Aucun fichier HTML

Vérifier que le programme a été lancé depuis `simulation` et qu'il s'est
terminé sans erreur.

### La page est lente

Réduire la durée, augmenter raisonnablement le pas de temps ou tester moins de
scénarios. Les longues simulations produisent beaucoup d'états avant
sous-échantillonnage.

### Le bateau paraît dériver ou viser à côté

Vérifier d'abord :

- la convention de direction du vent ;
- l'ordre latitude/longitude ;
- le cap initial ;
- la durée disponible pour atteindre la cible ;
- le fait qu'un zigzag peut être normal ;
- les limites du modèle décrites dans `CODE_ARCHITECTURE.md`.

Une petite dérive est volontaire : le modèle MEA ajoute une composante latérale
due au vent apparent. La navigation utilise aussi un GPS et un cap simulés avec
du bruit. Un scénario qui fonctionnait avec l'ancien modèle idéal peut donc
révéler une faiblesse de correction de route.

### Un numéro de scénario ne sélectionne rien

C'est le comportement actuel : les arguments de l'exécutable ne sont pas
traités. Pour isoler temporairement un scénario, commenter ses autres appels
dans `main()`, puis recompiler.
