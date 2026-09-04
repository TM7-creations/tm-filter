# TM IMU Orientation Filter

Filtre d’orientation IMU en C99 fondé sur les quaternions. Il fusionne un gyroscope et un accéléromètre sans magnétomètre et propose deux intégrations du gyroscope : une solution analytique par intervalle et une approximation d’Euler normalisée, plus légère.

La correction accélérométrique est calculée directement : parmi les orientations compatibles avec la direction mesurée de la gravité, le filtre choisit celle qui est la plus proche de la prédiction gyroscopique. Cette projection analytique remplace l’étape de descente de gradient employée dans le filtre de Madgwick.

> **État du projet : prototype expérimental.** Les propriétés mathématiques et numériques du code sont testées ([détail des vérifications](docs/validation.md)). La précision absolue n’a pas été mesurée avec un système de référence externe. Le démonstrateur Madgwick permet une comparaison visuelle, pas une validation métrologique.

## Fonctionnement

### 1. Prédiction par le gyroscope

Le quaternion suit, avec la convention de ce projet :

$$
\dot q=\frac{1}{2}q\otimes[0,\omega_x,\omega_y,\omega_z].
$$

En considérant la vitesse angulaire constante entre deux échantillons, on obtient la mise à jour analytique suivante :

$$
q_{k+1}=q_k\cos(x)+\frac{q_k\otimes[0,\boldsymbol\omega]}{\|\boldsymbol\omega\|}\sin(x),
\qquad x=\frac{\|\boldsymbol\omega\|\Delta t}{2}.
$$

`tm_gyro_update_exact()` calcule directement cette solution à l’aide des fonctions sinus et cosinus. La vitesse utilisée est actualisée à chaque mesure : elle peut donc varier d’un intervalle au suivant.

`tm_gyro_update_fast()` calcule Euler, puis normalise :

$$
q_{k+1}=\operatorname{normalize}\!\left(q_k+\frac{\Delta t}{2}q_k\otimes[0,\boldsymbol\omega]\right).
$$

Les deux fonctions partent de la même équation continue, mais elles ne sont pas strictement équivalentes pour un pas fini. L’angle d’Euler normalisé vaut $2\arctan(x)$, alors que l’angle exact vaut $2x$. Leur écart devient très faible lorsque $\|\boldsymbol\omega\|\Delta t$ est petit.

| Méthode | Atout | Limite |
|---|---|---|
| `EXACT` | Intégration analytique exacte du modèle retenu | Appelle `sinf()` et `cosf()` |
| `FAST` | Calcul plus simple | Erreur de discrétisation dépendant de la vitesse et du pas |

La méthode par défaut est `FAST`. Elle se sélectionne à l’exécution dans `filter.config.gyro_method`, ou à la compilation avec `-DTM_GYRO_METHOD=TM_GYRO_METHOD_EXACT`.

### 2. Correction analytique par l’accéléromètre

Lorsque les accélérations dues au mouvement sont négligeables, les mesures de l’accéléromètre permettent de déterminer la direction de l’axe vertical du repère monde dans le repère du capteur. Dans les conventions du projet, cette direction est donnée par le vecteur normalisé :

$$
\hat a=\frac{(a_x,a_y,a_z)}{\sqrt{a_x^2+a_y^2+a_z^2}}.
$$

Cette information fixe l’inclinaison du capteur, mais ne détermine pas une orientation complète. Plusieurs orientations possèdent la même verticale : elles diffèrent par une rotation autour de l’axe vertical du repère monde.

Le filtre utilise la prédiction gyroscopique pour choisir ce degré de liberté restant. Il cherche, **parmi tous les quaternions compatibles avec la verticale mesurée, celui qui est le plus proche du quaternion prédit par le gyroscope**. Il corrige ainsi l’inclinaison en introduisant le plus petit changement d’orientation nécessaire pour satisfaire la mesure.

En notant $q_g$ la prédiction gyroscopique et $R(q)$ la rotation du repère capteur vers le repère monde, le problème s’écrit :

$$
q_a=\underset{\|q\|=1,\;R(q)\hat a=e_z}{\operatorname{argmin}}
\;2\arccos\!\left(|\langle q_g,q\rangle|\right),
\qquad e_z=(0,0,1).
$$

**La solution de ce problème est calculée analytiquement**, sans itérations de descente de gradient. La fonction `tm_accel_quaternion()` détermine directement le quaternion $q_a$ qui satisfait la mesure et minimise cet écart. Ce quaternion constitue la cible de correction ; l’étape de fusion règle ensuite l’importance qu’on lui accorde.

Avant d’utiliser cette cible, le filtre vérifie que la mesure accélérométrique est acceptable : sa norme doit rester proche de celle attendue au repos, et sa direction ne doit pas varier trop rapidement entre deux mesures. Ces contrôles réduisent l’influence de certaines accélérations liées au mouvement, sans pouvoir toutes les identifier.

Les seuils se règlent avec `accel_tolerance` et `max_accel_direction_rate`. La première mesure valide utilise uniquement le test de norme ; mettre `max_accel_direction_rate = 0` désactive le test de variation de direction.

### 3. Fusion

Le quaternion corrigé est rapproché du quaternion gyroscopique par une interpolation linéaire normalisée suivant le chemin de signe le plus court. La loi conservée du prototype est :

$$
\text{poids}=\operatorname{clamp}(k_f\,\alpha,0,1),
$$

où $\alpha$ est l’écart angulaire en radians et `fusion_gain` vaut `0.1` par défaut. Ainsi, `fusion_gain` est un gain en radian⁻¹ ; ce n’est pas directement un pourcentage fixe de confiance.

## Conventions et unités

- quaternion de Hamilton ordonné `[w, x, y, z]` ;
- orientation du repère mobile **corps** vers le repère **monde** ;
- produit cinématique `q ⊗ [0, ω]` ;
- gyroscope exprimé en **rad/s**, dans le repère corps ;
- accéléromètre exprimé en **m/s²**, dans le repère corps ;
- au repos, repères alignés et capteur placé face vers le haut : `[ax, ay, az] ≈ [0, 0, +9.80665]` ;
- `dt` en secondes et strictement positif.

Si le montage physique utilise d’autres directions d’axes, appliquez une transformation cohérente au gyroscope et à l’accéléromètre avant d’appeler le filtre.

## Arborescence

```text
include/tm_filter.h                 API publique
src/tm_filter.c                     filtre
examples/basic_example.c            exemple minimal sans matériel
examples/arduino/mpu6050_stream/     acquisition MPU-6050 à 100 Hz
examples/desktop/imu_viewer.c        visualisation raylib
examples/desktop/madgwick.c          comparaison Madgwick réorganisée
tests/test_filter.c                  tests mathématiques et série
docs/                                protocole, migration et dérivation
```

## Construire et tester

Prérequis : un compilateur C99 et CMake 3.16 ou supérieur.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
./build/basic_example
```

Pour vérifier la méthode exacte comme valeur par défaut :

```bash
cmake -S . -B build-exact -DTM_GYRO_METHOD=EXACT
cmake --build build-exact
ctest --test-dir build-exact --output-on-failure
```

### Intégrer la bibliothèque

Copiez `include/tm_filter.h` et `src/tm_filter.c`, puis liez la bibliothèque mathématique si la plateforme l’exige (`-lm`).

```c
#include "tm_filter.h"

tm_filter filter;
tm_filter_init(&filter);
filter.config.gyro_method = TM_GYRO_METHOD_EXACT;

int result = tm_filter_update(
    &filter,
    gx_rad_s, gy_rad_s, gz_rad_s,
    ax_m_s2, ay_m_s2, az_m_s2,
    dt_seconds
);

if (result >= 0) {
    /* filter.q contient [w, x, y, z].
       result == 1 : correction accélérométrique appliquée.
       result == 0 : prédiction gyroscopique seule. */
}
```

Chaque objet `tm_filter` possède son propre état : plusieurs capteurs peuvent donc être traités sans variables globales partagées.

## Démonstrateur MPU-6050 et raylib

Le sketch utilise uniquement la bibliothèque `Wire`, fournie avec Arduino. Sur une Uno :

| Connexion | Broche Uno |
|---|---|
| SDA du module MPU-6050 | A4 / SDA |
| SCL du module MPU-6050 | A5 / SCL |
| Masse du module | GND |
| Bouton de calibration | entre D2 et GND |

L’alimentation dépend du module utilisé ; suivez sa documentation. L’adresse I²C configurée est `0x68` (AD0 bas). Le sketch configure explicitement les plages ±2 g et ±2000 °/s.

Le filtre s’exécute sur l’ordinateur dans cet exemple ; l’Arduino acquiert et transmet les mesures. La bibliothèque C peut aussi être intégrée dans un programme embarqué.

1. Ouvrez `examples/arduino/mpu6050_stream/mpu6050_stream.ino` dans l’IDE Arduino et téléversez-le sur la carte.
2. Vérifiez le montage et adaptez `ACCEL_BIAS` ainsi que les axes à votre capteur.
3. Installez [raylib](https://www.raylib.com/) pour votre plateforme.
4. Construisez le visualiseur :

```bash
cmake -S . -B build-viewer -DTM_BUILD_VIEWER=ON
cmake --build build-viewer
```

5. Repérez le port série et lancez, par exemple sur macOS :

```bash
ls /dev/cu.*
./build-viewer/imu_viewer /dev/cu.usbserial-XXXX exact 100
```

Maintenez le bouton relié à la broche 2 pendant que le capteur est immobile, puis relâchez-le. Cela calcule le biais du gyroscope en RAM et réinitialise les orientations. Les trois repères sont positionnés en `x = −1.5` (TM), `x = 0` (fixe) et `x = +1.5` (Madgwick). Leurs positions à l’écran dépendent de la perspective.

Le protocole encode explicitement ses valeurs en little-endian et utilise des `float` IEEE-754 de 32 bits. Le décodeur reconstruit les valeurs indépendamment de l’ordre des octets de l’hôte. Il ne contient pas de somme de contrôle. Voir [la description du protocole](docs/serial-protocol.md).

## Paramètres par défaut

| Paramètre | Valeur | Sens |
|---|---:|---|
| `gyro_method` | `FAST` | méthode d’intégration du gyroscope |
| `fusion_gain` | `0.1 rad⁻¹` | force de rappel proportionnelle à l’écart |
| `gravity` | `9.80665 m/s²` | norme attendue au repos |
| `accel_tolerance` | `0.5 m/s²` | tolérance absolue sur la norme |
| `max_accel_direction_rate` | `0.4 rad/s` | variation maximale de direction acceptée |

Ces valeurs viennent du prototype et constituent un point de départ. Elles doivent être réglées et validées pour le capteur, la fréquence, les vibrations et la dynamique de l’application.

## Comparaison avec Madgwick

La différence principale porte sur le calcul de la correction accélérométrique. Dans la formulation IMU de [Madgwick](https://x-io.co.uk/downloads/madgwick_internal_report.pdf), un gradient calculé analytiquement fournit une direction de correction. Cette correction, pondérée par le gain `beta`, est incorporée à la dérivée du quaternion avant intégration.

Le filtre TM calcule directement **la solution exacte du problème géométrique de correction** : trouver le quaternion qui satisfait la verticale mesurée tout en restant le plus proche de la prédiction gyroscopique. Il n’a pas besoin de s’approcher de cette cible par des pas de descente de gradient. La fusion pondère ensuite cette cible en fonction de l’écart angulaire.

Cet avantage concerne la résolution du problème de correction. Il ne suffit pas, à lui seul, à démontrer une estimation d’orientation plus précise que Madgwick sur des mesures réelles : le bruit, les accélérations linéaires, les biais et le réglage des gains influencent aussi l’erreur finale. Une cible parfaitement compatible avec une mesure perturbée peut elle-même s’écarter de l’orientation réelle.

L’intégration gyroscopique `EXACT` supprime également l’erreur de discrétisation d’Euler pour le modèle à vitesse constante par intervalle. La version Madgwick incluse dans ce dépôt utilise Euler ; cet avantage ne s’applique donc à la comparaison gyroscopique que lorsque TM utilise le mode `EXACT`.

Le fichier `madgwick.c` est une implémentation de comparaison réorganisée à partir du code expérimental du projet. Son gain `beta = 0.1` ne correspond pas au gain `fusion_gain = 0.1` de TM : les lois et les unités diffèrent. Pour mesurer un éventuel gain de précision global, il faut comparer les deux filtres sur les mêmes données, avec des réglages adaptés et une orientation de référence indépendante.

## Limites

- Sans magnétomètre ni autre référence de cap, le lacet et sa dérive ne peuvent pas être corrigés.
- Une accélération linéaire proche de `1 g` peut franchir le test de norme et perturber l’inclinaison.
- Le modèle de propagation ne décrit pas les variations de vitesse à l’intérieur d’un intervalle d’échantillonnage.
- La fusion dépend de la fréquence : sa loi n’est pas un gain continu multiplié par `dt`.
- Le seuil de direction par défaut (`0.4 rad/s`, environ 23 °/s) est restrictif et peut refuser des rotations réelles ; augmentez-le ou mettez-le à zéro selon l’application.
- Le démonstrateur reconstruit `dt` à partir du compteur et de la fréquence nominale ; il ne mesure pas la gigue d’échantillonnage.
- Les axes, biais et facteurs d’échelle doivent être calibrés sur le matériel réel.

## Références

- S. O. H. Madgwick, [*An efficient orientation filter for inertial and inertial/magnetic sensor arrays*](https://x-io.co.uk/downloads/madgwick_internal_report.pdf), 2010.
- TDK InvenSense, [*MPU-6000/MPU-6050 Register Map and Descriptions*](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf).
- [Documentation raylib](https://www.raylib.com/cheatsheet/cheatsheet.html).

## Licence

Le code propre au projet est proposé sous [licence MIT](LICENSE), avec les exclusions précisées dans [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

La licence autorise l’utilisation, la modification et la redistribution du code couvert, y compris dans des projets commerciaux, à condition de conserver la notice de copyright et la licence. Elle n’oblige pas les utilisateurs à publier leurs modifications et prévoit une fourniture sans garantie. Le texte anglais du fichier `LICENSE` fait référence.

La notice de copyright utilise provisoirement le nom « TM », issu du nom du filtre. L’auteur peut le remplacer par le nom ou le pseudonyme qu’il souhaite utiliser pour s’identifier.

**Les fichiers de comparaison `madgwick.c` et `madgwick.h` sont exclus de cette attribution de licence MIT.** La provenance du code initial doit être clarifiée avant leur publication ; les éventuelles notices et conditions d’origine devront être conservées.
