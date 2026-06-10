# Architecture du simulateur

Ce document explique comment le simulateur est construit. Il s'adresse à une
personne qui découvre le projet et ne suppose aucune connaissance préalable de
la navigation à voile.

Pour utiliser le programme, voir [UTILISATION.md](UTILISATION.md).
Pour comprendre les décisions de navigation, voir [NAVIGATION.md](NAVIGATION.md).

## 1. À quoi sert le simulateur ?

Le simulateur permet de tester la logique de navigation sans bateau réel. Il
fait tourner une boucle qui répète deux opérations :

1. la navigation choisit une position de voile et une position de gouvernail ;
2. un modèle physique simplifié calcule le nouveau déplacement du bateau.

Les positions successives sont mémorisées puis exportées dans une page HTML
interactive.

Le simulateur n'est pas un modèle marin réaliste. Son objectif principal est de
vérifier les décisions générales : rejoindre un point, tirer des bords face au
vent, éviter un empannage direct et passer au point suivant.

## 2. Organisation des fichiers

```text
simulation/
├── Makefile
├── sim_main.cpp                 scénarios et point d'entrée
├── sim_boat.hpp/.cpp            adaptation de la navigation au simulateur
├── sim_environment.hpp/.cpp     état et physique simplifiée du bateau
├── exporter.hpp/.cpp            création de la page HTML
├── mock/
│   ├── Arduino.h/.cpp           remplacement minimal d'Arduino
│   └── LoRa.h/.cpp              remplacement minimal de LoRa
└── docs/
    ├── CODE_ARCHITECTURE.md
    ├── UTILISATION.md
    └── NAVIGATION.md

main/src/navigation/
├── navigation.h                 navigation actuelle
├── oldNavigation.h              ancienne navigation
├── NavigationSelector.h         sélection utilisée par le firmware
└── NavigationConfig.h           choix entre les deux versions
```

## 3. Vue d'ensemble

```text
sim_main.cpp
    │ crée un scénario
    ▼
SimulatedBoat
    │ fournit position, cap, vent et waypoint
    ▼
navigation.h
    │ renvoie voile, gouvernail, mode et arrivée éventuelle
    ▼
SimulationEnvironment
    │ calcule vitesse, rotation et nouvelle position
    ▼
historique des SimBoatState
    │
    ▼
exporter.cpp → output/simulation.html
```

Les responsabilités sont volontairement séparées :

| Élément | Responsabilité |
|---|---|
| `sim_main.cpp` | Décrire les scénarios et leur durée |
| `SimulatedBoat` | Relier la navigation réelle à la simulation |
| `SimulationEnvironment` | Faire évoluer le bateau dans le temps |
| `navigation.h` | Décider comment diriger le bateau |
| `exporter.cpp` | Produire la visualisation HTML |
| `mock/` | Remplacer les fonctions Arduino absentes sur un ordinateur |

## 4. Déroulement d'un scénario

Un scénario suit généralement cette séquence :

```cpp
SimTime::init();

SimulatedBoat boat;
boat.init(latitude, longitude, ventDirection, ventVitesse, capInitial);

boat.addWaypoint(latitudeCible, longitudeCible);

boat.startWindObservation();
boat.runSimulation(dureeObservationMs, pasDeTempsMs);

boat.startNavigation();
boat.runSimulation(dureeNavigationMs, pasDeTempsMs);
```

### Initialisation

`boat.init(...)` initialise :

- la position GPS ;
- le cap du bateau ;
- la direction et la vitesse du vent ;
- les angles initiaux de voile et de gouvernail ;
- l'état interne de la navigation.

### Ajout des waypoints

Un waypoint est une cible GPS :

```cpp
boat.addWaypoint(48.342791, -4.531538);
```

Le rayon d'arrivée n'est pas un argument de `addWaypoint(...)`. Il est fixé à
`10 m` par `WAYPOINT_DISTANCE_SIM` dans `sim_boat.hpp`. Le waypoint est
considéré atteint quand le bateau entre dans ce cercle, ou quand il dépasse la
ligne d'arrivée dans le corridor autorisé.

Les waypoints sont parcourus dans leur ordre d'ajout.

### Observation du vent

`startWindObservation()` place le bateau dans une phase particulière. Le
simulateur attend que le bateau se soit éloigné de `30 m` de son point de
départ, puis estime la direction du vent à partir de son cap. Cette phase
suppose initialement un vent relatif de `+90°` et force donc la commande du
safran à `+45°`. Après acquisition, la compensation est recalculée avec le vent
relatif estimé.

### Navigation

`startNavigation()` active les décisions de navigation. À chaque pas de temps,
la navigation reçoit notamment :

- la position actuelle ;
- le cap actuel ;
- la direction du vent ;
- le waypoint courant ;
- les angles de voile et de gouvernail ;
- son état interne précédent.

Elle renvoie un `NavResult` contenant les nouvelles commandes et le mode choisi.

## 5. Une itération de simulation

La méthode centrale est `SimulatedBoat::stepSimulation(...)`.

```text
1. Lire l'état actuel
2. Appeler la logique de navigation
3. Appliquer les commandes de voile et de gouvernail
4. Calculer le mouvement pendant le pas de temps
5. Enregistrer le nouvel état dans l'historique
6. Passer au waypoint suivant si nécessaire
```

Cette distinction est importante :

- la navigation décide ce que le bateau devrait faire ;
- l'environnement décide ce que ces commandes produisent physiquement.

Une erreur de trajectoire peut donc venir de l'une ou de l'autre partie.

## 6. État simulé

`SimBoatState` regroupe les valeurs nécessaires à un instant donné :

| Champ | Signification | Unité |
|---|---|---|
| `latitude`, `longitude` | Position GPS | degrés |
| `heading` | Cap du bateau | degrés |
| `speed` | Vitesse | m/s |
| `sailAngle` | Déflexion de l'aileron de voile | degrés |
| `rudderAngle` | Commande servo : compensation + correction | degrés |
| `windDirection` | Direction d'où vient le vent | degrés |
| `windSpeed` | Vitesse du vent | m/s |
| `time` | Temps simulé | ms |
| `navMode` | Décision de navigation affichée | entier |

Cet état représente le bateau physique et sert à l'export. La navigation ne lit
pas directement ces valeurs parfaites. Elle reçoit un `SimSensorState` avec :

- une position GPS perturbée d'environ `±1,5 m` ;
- un cap légèrement bruité ;
- un bruit de cap plus important quand la vitesse devient faible.

Les angles de cap suivent la convention d'une boussole :

```text
0° ou 360° : nord
90°         : est
180°        : sud
270°        : ouest
```

La direction du vent indique d'où vient le vent. Un vent de `270°` vient donc
de l'ouest et souffle vers l'est.

## 7. Modèle physique

Le modèle physique se trouve dans
`SimulationEnvironment::updateBoatDynamics(...)`. Il est volontairement simple.

### Vent apparent

Le modèle combine le vent réel et le déplacement du bateau :

```text
vecteur de vent apparent =
    vecteur du vent réel - vecteur vitesse du bateau
```

Toutes les décisions aérodynamiques suivantes utilisent ce vent apparent. Le
vent ressenti change donc quand le bateau accélère ou change de cap.

### Vitesse produite par l'aile

Le programme calcule l'angle entre le bateau et le vent. Une polaire simplifiée
attribue ensuite un coefficient de vitesse :

- presque face au vent : très peu de propulsion ;
- vent de travers : propulsion importante ;
- vent arrière : propulsion possible mais réduite près de l'axe interdit.

La vitesse cible est limitée à environ `2,5 m/s`. La vitesse réelle rejoint
progressivement cette cible avec une constante de temps de `2 s` en accélération
et `6 s` en décélération.

### Compensation mécanique et gouvernail

La liaison mécanique place naturellement le safran à l'opposé de la moitié du
vent relatif estimé. La navigation commande la compensation inverse, puis
ajoute sa correction :

```text
liaison mécanique = -vent relatif / 2
commande servo = +vent relatif / 2 + correction
safran physique = liaison mécanique + commande servo
                = correction
```

Le rapport `2:1` cherche à compenser approximativement le moment créé par
l'aile. En mode direct, la correction vaut zéro : le servo annule la position
imposée par la liaison et le safran physique revient approximativement dans
l'axe.

La position physique du safran rejoint sa cible avec une constante de temps de
`0,7 s`. Sa partie de base est considérée comme compensée par l'aile. L'écart
autour de cette base produit la rotation, avec une efficacité réduite à faible
vitesse. La rotation du bateau possède ensuite une inertie de `1 s`.

Le cap est ensuite normalisé entre `0°` et `360°`.

Le bateau réel comporte une liaison mécanique `2:1` inversée entre le vent
relatif à l'aile et le safran. Avec une correction de navigation nulle, le servo
commande quand même `vent relatif / 2` pour annuler cette liaison et replacer le
safran physique dans l'axe. Si le vent relatif change rapidement, le retard
mécanique du safran peut toutefois créer une rotation transitoire.

Dans la page HTML, l'axe rouge représente la position physique complète :

```text
angle physique du safran =
    -vent relatif / 2 + commande servo
```

La jauge « Commande servo safran » affiche la compensation et la correction.

### Dérive et déplacement GPS

Le déplacement comporte une composante vers l'avant et une dérive latérale
créée par la composante transversale du vent apparent :

```text
vitesse sol =
    vitesse avant selon le cap
    + vitesse de dérive latérale
```

La dérive est limitée à `0,35 m/s`. Les deux composantes sont projetées vers le
nord et l'est, puis converties en latitude et longitude.

## 8. Temps simulé

Le simulateur utilise `SimTime` à la place de l'horloge réelle d'Arduino. Il
peut donc simuler plusieurs jours sans attendre plusieurs jours devant
l'ordinateur.

Le pas de temps contrôle la fréquence des calculs :

```cpp
boat.runSimulation(60 * 60 * 1000UL, 100);
```

Cet exemple simule une heure avec un calcul toutes les `100 ms`.

Les inerties physiques utilisent une réponse exponentielle liée au temps. Elles
restent donc comparables quand le pas change. Un petit pas donne toutefois une
trajectoire et des décisions de navigation plus détaillées, tandis qu'un grand
pas réduit la précision de la boucle de contrôle.

## 9. Historique et export HTML

Après chaque déplacement, l'état est ajouté à un historique. L'exporteur
rassemble ensuite les historiques de tous les scénarios dans :

```text
simulation/output/simulation.html
```

La page contient :

- une carte et la trajectoire ;
- les waypoints ;
- une animation de la progression ;
- le cap, la vitesse, la voile et le gouvernail ;
- le mode de navigation courant ;
- un onglet par scénario.

Les très longs historiques sont sous-échantillonnés pour que la page reste
utilisable. L'HTML charge Leaflet et les fonds de carte depuis Internet.

## 10. Relation avec le firmware réel

Le cœur de navigation actuel est partagé : le simulateur inclut directement
`main/src/navigation/navigation.h`.

Le firmware, lui, passe par `NavigationSelector.h`, qui sélectionne la version
actuelle ou l'ancienne version selon `USE_OLD_NAVIGATION`.

```text
Firmware : NavigationSelector.h → navigation.h ou oldNavigation.h
Simulation : navigation.h directement
```

Conséquence : changer `USE_OLD_NAVIGATION` ne change pas automatiquement la
version testée par le simulateur.

Le simulateur ne reproduit pas non plus toute l'application embarquée. Il
remplace notamment la gestion de mission, les capteurs, les servomoteurs, les
communications et les erreurs matérielles par des équivalents simplifiés.

## 11. Limites connues

Il faut garder ces limites en tête lors de l'interprétation :

- pas de courant marin, vagues ni rafales ;
- dérive latérale simplifiée, sans quille ni hydrodynamique détaillée ;
- bruit capteur déterministe, pas issu d'un modèle de récepteur GPS réel ;
- réponse de voile et de gouvernail simplifiée ;
- vitesse maximale arbitraire ;
- les commandes de navigation par pas de `5°` dépendent encore de la fréquence
  d'appel, même si les inerties physiques dépendent maintenant du temps ;
- le rapport `2:1`, les inerties et le gain de rotation doivent être calibrés
  sur le MEA réel ;
- la simulation teste directement `navigation.h`, pas le sélecteur du firmware ;
- atteindre un waypoint en simulation ne garantit pas le même résultat en mer.

## 12. Ordre de lecture conseillé

Pour découvrir le code :

1. lire un scénario dans `sim_main.cpp` ;
2. lire l'interface de `sim_boat.hpp` ;
3. suivre `SimulatedBoat::stepSimulation(...)` ;
4. lire `SimulationEnvironment::updateBoatDynamics(...)` ;
5. utiliser [NAVIGATION.md](NAVIGATION.md) avant d'ouvrir `navigation.h` ;
6. terminer par `exporter.cpp`, qui concerne surtout l'affichage.
