# Comprendre la logique de navigation

Ce document explique comment le bateau choisit sa trajectoire. Il commence par
les notions de voile nécessaires, puis relie chaque notion au code actuel dans
`main/src/navigation/navigation.h`.

Pour lancer les scénarios, voir [UTILISATION.md](UTILISATION.md). Pour comprendre
le reste du simulateur, voir [CODE_ARCHITECTURE.md](CODE_ARCHITECTURE.md).

## 1. Le problème à résoudre

À chaque décision, le bateau connaît principalement :

- sa position GPS ;
- son cap ;
- la direction estimée du vent ;
- la position du waypoint à rejoindre ;
- la position actuelle de la voile et du gouvernail ;
- l'état conservé depuis la décision précédente.

Il doit choisir :

- un angle de voile ;
- un angle de gouvernail ;
- un délai avant la prochaine décision ;
- un mode de navigation ;
- si le waypoint est atteint.

La navigation ne déplace pas elle-même le bateau. Elle donne des ordres. Dans
la simulation, `SimulationEnvironment` transforme ensuite ces ordres en
mouvement.

## 2. Vocabulaire minimal

### Cap

Le cap est la direction vers laquelle pointe l'avant du bateau :

```text
        0° nord
           ↑
270° ouest ← → 90° est
           ↓
       180° sud
```

### Waypoint

Un waypoint est une cible GPS. La navigation calcule le relèvement, c'est-à-dire
la direction depuis le bateau vers cette cible.

### Direction du vent

Dans ce projet, la direction indique d'où vient le vent.

```text
vent de 270° : il vient de l'ouest et souffle vers l'est
```

Cette convention est essentielle. L'interprétation inverse produit une
navigation incohérente.

### Gouvernail

Le gouvernail fait tourner le bateau. La navigation limite la correction de cap
à `-20°` / `+20°`, puis lui ajoute la compensation `vent relatif / 2`.

### Voile

La voile transforme le vent en propulsion. La logique actuelle utilise surtout
deux positions simplifiées, `-10°` et `+10°`, choisies selon le côté du vent.

### Lofer et abattre

- **Lofer** : tourner vers la direction d'où vient le vent.
- **Abattre** : tourner en s'éloignant de cette direction.

### Virement et empannage

- **Virement de bord** : l'avant du bateau traverse l'axe du vent.
- **Empannage** : l'arrière du bateau traverse l'axe du vent.

Dans la logique actuelle, l'empannage n'est pas exécuté. Il est évité, car la
voile pourrait changer de côté brutalement lorsque l'arrière du bateau traverse
l'axe du vent.

## 3. Pourquoi le bateau ne va pas toujours tout droit

Un voilier ne peut pas progresser efficacement dans toutes les directions.

### Zone interdite face au vent

La navigation considère une zone de `45°` de chaque côté de l'origine du vent.
Un waypoint situé dans cette zone ne peut pas être visé directement.

```text
                 origine du vent
                        ↑
                   zone interdite
                      /   \
                     /     \
                    ⛵
```

Le bateau choisit alors un cap praticable sur un côté, puis alterne les côtés :
il tire des bords.

### Zone sensible sous le vent

Une zone de `20°` autour de la direction opposée au vent est également traitée
à part. Elle évite de rester exactement vent arrière et sert à déclencher la
procédure d'évitement lorsqu'un changement de côté serait nécessaire.

## 4. Angles relatifs

Le code convertit les directions absolues en angles relatifs compris entre
`-180°` et `+180°`.

```text
angle relatif = direction cible - cap actuel
```

Après normalisation :

- angle proche de `0°` : cible presque droit devant ;
- angle positif : cible d'un côté du bateau ;
- angle négatif : cible de l'autre côté ;
- valeur proche de `180°` ou `-180°` : cible derrière.

La fonction centrale est :

```cpp
nav_relativeAngle(from, to)
```

Cette représentation permet de choisir le sens de rotation le plus court sans
se tromper au passage entre `359°` et `0°`.

## 5. Résultat et mémoire de navigation

### `NavResult`

Chaque appel renvoie un résultat contenant notamment :

| Champ | Rôle |
|---|---|
| `sailAngle` | commande de voile |
| `rudderAngle` | commande servo : vent relatif / 2 + correction |
| `sendInterval` | délai proposé avant un nouvel envoi |
| `mode` | nom court de la décision |
| `logMessage` | explication destinée au diagnostic |
| `windAcquired` | observation du vent terminée |
| `waypointReached` | cible courante atteinte |

### `NavState`

Certaines décisions durent plusieurs appels. `NavState` mémorise notamment :

- le corridor associé au waypoint courant ;
- le côté choisi pour les bords face au vent ;
- le côté choisi sous le vent ;
- l'étape en cours de la procédure d'évitement d'empannage ;
- la cible d'une manœuvre.

Sans cet état, le bateau pourrait changer d'avis à chaque itération et osciller
rapidement entre deux décisions.

## 6. Ordre des décisions

`nav_handleNavigationWithState(...)` suit cet ordre général :

```text
Waypoint atteint ?
    oui → signaler l'arrivée
    non
      ↓
Évitement d'empannage déjà commencé ?
    oui → poursuivre ses étapes
    non
      ↓
Waypoint face au vent ou rotation traversant le vent ?
    oui → mode face au vent, tirer des bords
    non
      ↓
Waypoint sous le vent ?
    oui → mode vent arrière contrôlé
    non
      ↓
Rotation directe traversant l'axe arrière ?
    oui → commencer la procédure d'évitement
    non
      ↓
Cible presque devant ?
    oui → tout droit
    non → lofer ou abattre
```

Cet ordre compte : une manœuvre déjà engagée est terminée avant de reprendre une
décision normale.

## 7. Navigation directe

Dans les modes de manœuvre, `nav_applyTargetHeading(...)` calcule :

```text
erreur = différence entre cap actuel et cap cible
commande de gouvernail = erreur × gain
```

La correction est limitée à `20°`. La navigation lui ajoute ensuite la
compensation `vent relatif / 2` pour produire la commande servo totale.

Hors des zones interdites, si le waypoint est à moins de `5°` de l'avant, le
bateau est considéré correctement aligné. La correction revient à zéro, mais
la commande servo conserve la compensation `vent relatif / 2`.
Sinon, le code ne vise pas directement le relèvement avec cette formule : il
choisit de lofer ou d'abattre et modifie la commande de gouvernail par pas de
`5°`.

## 8. Navigation face au vent

Si le waypoint se trouve dans la zone interdite face au vent, la navigation
construit deux caps possibles :

```text
cap bâbord  = direction du vent - 45°
cap tribord = direction du vent + 45°
```

Elle choisit d'abord le cap qui fait réellement progresser vers le waypoint.
Ce point évite de choisir un bord uniquement parce qu'il demande une petite
rotation alors qu'il éloignerait le bateau de sa cible.

Le bateau conserve ensuite ce côté jusqu'à sortir de son corridor. À ce moment,
il change de bord pour revenir vers la route générale.

La trajectoire attendue ressemble à ceci :

```text
                waypoint
                   ▲
          / \      │
         /   \     │ route générale
        /     \    │
       ⛵
```

## 9. Le corridor de route

Lorsqu'un nouveau waypoint devient actif, le code mémorise :

- la position de départ du segment ;
- la position du waypoint ;
- la direction et la longueur du segment.

Cela définit un corridor autour de la route idéale :

```text
limite gauche  +--------------------------+
               | départ ---------- cible |
limite droite  +--------------------------+
```

La demi-largeur actuelle vaut `100 m`. Le bateau peut donc s'écarter de part et
d'autre, mais un écart plus important déclenche un choix de bord destiné à le
ramener vers la route.

La fonction `nav_crossTrackErrorMeters(...)` calcule la distance signée à cette
route :

- le signe indique le côté ;
- la valeur absolue indique l'écart en mètres.

Le corridor évite deux problèmes :

- changer de bord beaucoup trop souvent ;
- continuer longtemps dans une direction qui éloigne de la route.

## 10. Détection de l'arrivée

Le cas simple est :

```text
distance au waypoint ≤ rayon d'arrivée
```

Dans le simulateur, ce rayon est fixé à `10 m` par
`WAYPOINT_DISTANCE_SIM` dans `sim_boat.hpp`.

Le code traite aussi le cas où le bateau dépasse légèrement la cible entre deux
calculs. Le waypoint est alors atteint si :

1. la projection du bateau a dépassé le plan perpendiculaire à la fin du
   segment ;
2. le bateau reste dans la largeur du corridor.

```text
route -------- waypoint | plan d'arrivée
                         |     ⛵ dépassé
```

Cette règle évite qu'un grand pas de temps oblige le bateau à faire demi-tour
pour quelques mètres.

## 11. Navigation sous le vent

Quand la cible est trop proche de l'axe exactement opposé au vent, le bateau
choisit un cap décalé de `20°`.

Comme face au vent, il conserve un côté et utilise le corridor pour décider
quand changer. L'objectif est d'avancer vers la cible sans rester sur un axe où
la voile et le changement de côté deviennent difficiles à contrôler.

## 12. Procédure d'évitement d'empannage

Si le chemin de rotation le plus court traverse l'axe arrière du vent, la
navigation refuse cette rotation directe. Elle lance une machine à états qui
fait passer le bateau par l'axe face au vent.

La route d'évitement comporte trois étapes :

1. rejoindre la limite de la zone interdite face au vent ;
2. traverser l'axe du vent avec l'avant du bateau, comme pendant un virement ;
3. revenir vers la limite sous le vent du côté souhaité.

```text
                             vent
                               ↑
                 étape 2  ←  ⛵  →  passage par l'avant
                            /     \
                  étape 1 /       \ étape 3
                         /         \
                  ancien côté     nouveau côté
                         axe vent arrière
```

`NavState` conserve la phase courante jusqu'à ce que chaque cap intermédiaire
soit atteint. Le bateau contourne ainsi la direction vent arrière au lieu de la
traverser avec sa poupe.

Les noms internes `empannagePhase`, `nav_handleEmpannageLoop(...)` et
`NAV_EMPANNAGE_...` peuvent prêter à confusion. Dans le comportement actuel,
ils représentent les phases de cet évitement, pas l'exécution d'un empannage.

Un cap temporairement très différent du waypoint est donc volontaire : le
bateau termine d'abord la route d'évitement.

## 13. Choix de la voile

`nav_setSailForHeading(...)` compare le cap visé et la direction du vent. La
voile est placée d'un côté ou de l'autre avec une amplitude simplifiée de
`10°`.

Ce n'est pas un réglage continu et réaliste de voile. Le modèle cherche surtout
à conserver une convention cohérente entre :

- le côté d'où vient le vent ;
- le côté de la voile ;
- le modèle physique du simulateur.

## 14. Observation du vent

Avant la navigation, `nav_handleWindObservation(...)` attend une phase
d'observation. Dans le simulateur, l'estimation est acquise lorsque le bateau
s'est éloigné de `30 m` de sa position de départ.

Au début de cette phase, le bateau est supposé placé avec le vent relatif à
`+90°`. La commande est donc forcée ainsi :

```text
aileron = +10°
commande servo safran = 90° / 2 = +45°
```

La méthode estime ensuite le vent avec `cap + 90°`. Dès que cette estimation est
acquise, la commande fixe de `45°` est remplacée par
`vent relatif calculé / 2`. Cette procédure reste simplifiée et suppose que le
placement initial à `90°` est respecté.

Une mauvaise estimation du vent décale toutes les zones interdites. Le bateau
peut alors tirer des bords au mauvais endroit ou tenter une route impossible.

## 15. Exemple concret

Supposons :

```text
cap actuel       : 90°  (est)
vent             : 0°   (vient du nord)
waypoint         : 10°  (presque face au vent)
```

Le waypoint est à moins de `45°` de l'origine du vent. La navigation ne vise
donc pas `10°` directement. Elle compare les deux caps praticables :

```text
315° = 0° - 45°
45°  = 0° + 45°
```

Elle choisit celui qui apporte la meilleure progression initiale, le conserve
jusqu'à la limite du corridor, puis change de côté. La trajectoire est plus
longue qu'une ligne droite, mais elle respecte la contrainte du voilier.

## 16. Paramètres principaux

Les constantes actuelles se trouvent au début de `navigation.h` :

| Paramètre | Valeur | Effet |
|---|---:|---|
| zone morte directe | `5°` | évite de corriger une petite erreur |
| gain direct | `0,5` | intensité du gouvernail en navigation normale |
| gain VDB | `0,8` | intensité pendant certains changements de bord |
| limite de correction | `20°` | correction maximale autour de la compensation |
| demi-largeur corridor | `100 m` | écart toléré autour de la route |
| zone face au vent | `45°` | angle interdit de chaque côté du vent |
| zone sous le vent | `20°` | angle évité autour du vent arrière |
| incrément lofer/abattre | `5°` | variation de gouvernail par décision |
| angle de voile | `10°` | position simplifiée de la voile |

Modifier ces valeurs change le comportement global. Il faut donc les tester sur
plusieurs situations, pas uniquement sur un trajet.

## 17. Version actuelle et ancienne version

Le firmware inclut `NavigationSelector.h`. La constante
`USE_OLD_NAVIGATION` dans `NavigationConfig.h` détermine la version utilisée :

```text
0 → navigation.h
1 → oldNavigation.h
```

Le simulateur inclut directement `navigation.h`. Il teste donc toujours la
version actuelle, indépendamment de `USE_OLD_NAVIGATION`.

## 18. Points de vigilance

La logique reste simplifiée et possède des limites :

- les commandes de lofer/abattre changent par appel, donc leur vitesse dépend de
  la fréquence d'exécution ;
- le vent observé est une estimation très simplifiée ;
- le réglage de voile ne représente pas une vraie optimisation ;
- le corridor est construit en approximation locale, adaptée aux trajets
  relativement courts ;
- le simulateur ne modélise pas le courant et les vagues ;
- le simulateur ajoute une dérive, un vent apparent et une compensation mécanique,
  avec un safran de base réglé à l'opposé de la moitié du vent relatif, mais les
  coefficients restent à calibrer sur le MEA réel ;
- la navigation reçoit un GPS et un cap simulés avec du bruit, particulièrement
  visible à faible vitesse ;
- la physique simulée et les capteurs réels peuvent encore réagir différemment ;
- `sendInterval` exprime une intention de rythme, mais la boucle du simulateur
  utilise son propre pas de temps.

Pour analyser une trajectoire surprenante, regarder dans cet ordre :

1. la direction réelle donnée au vent ;
2. le waypoint actif et son rayon ;
3. le mode de navigation affiché ;
4. le cap cible et le cap actuel ;
5. l'écart au corridor ;
6. l'état d'une éventuelle procédure d'évitement d'empannage ;
7. le comportement du modèle physique.

## 19. Ordre de lecture du code

Après ce document, l'ordre suivant limite les allers-retours :

1. `NavResult` et `NavState` ;
2. les fonctions de normalisation des angles ;
3. les calculs GPS et de corridor ;
4. `nav_applyTargetHeading(...)` ;
5. les fonctions face au vent et sous le vent ;
6. la machine à états d'évitement d'empannage ;
7. `nav_handleNavigationWithState(...)`, qui orchestre le tout.
