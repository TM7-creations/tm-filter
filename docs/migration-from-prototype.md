# Migration depuis le prototype

La version réorganisée se trouve dans un nouveau dossier afin de laisser intacts les fichiers de travail originaux. Une archive supplémentaire `imu-original.zip` est livrée à côté du dépôt.

## Renommages

| Prototype | Nouvelle API |
|---|---|
| `quaternion()` | `tm_gyro_update_fast()` |
| `quaternionV2()` commentée | `tm_gyro_update_exact()` |
| `acc()` | `tm_accel_quaternion()` |
| `fusion()` | `tm_quaternion_blend()` et loi dans `tm_filter_update()` |
| `TMfilter()` | `tm_filter_update()` |
| `TMunits()` / `set_units()` | conversions explicites avant l’appel |

Le nouvel objet `tm_filter` contient le quaternion, la configuration et l’historique de l’accéléromètre. Cela retire les paramètres d’unités globaux et autorise plusieurs instances indépendantes.

## Corrections effectuées

- normalisation après chaque fusion ;
- choix du signe quaternionique le plus court avant interpolation ;
- bornage des arguments trigonométriques ;
- gestion des accélérations nulles, des pôles et des nombres non finis ;
- test angulaire de variation de l’accéléromètre avec des unités cohérentes ;
- absence de mise à jour partielle lorsque le gyroscope, `dt`, le quaternion ou la configuration est invalide ;
- la comparaison Madgwick continue la prédiction gyroscopique lorsque son gradient est nul ou lorsque l’accélération est inutilisable ;
- inclusion d’un en-tête Madgwick au lieu d’inclure un fichier `.c` ;
- port série et fréquence passés en arguments ;
- décodeur indépendant, sans pointeurs non alignés ni dépendance à l’endian de l’hôte ;
- suppression des états partagés et du thread de lecture dans le visualiseur.

## Calibration et directions des axes

Le prototype Arduino appliquait des signes négatifs aux trois axes de l’accéléromètre ainsi que des offsets propres à un exemplaire :

```text
ax = -rawAx / 1670.13 + 0.09
ay = -rawAy / 1670.13 - 0.17
az = -rawAz / 1670.13 - 0.79
```

Ces valeurs ne sont pas transférables à un autre capteur. Le nouvel exemple emploie la convention standard `raw / 16384 × g` et des biais nuls clairement identifiés. Il faut mesurer les biais et adapter les signes ou permuter les axes selon le montage avant toute comparaison.

Le prototype transmettait aussi du texte pendant la calibration au milieu du flux binaire. Le nouvel exemple n’émet que des trames binaires afin de simplifier la resynchronisation.

## Différences numériques attendues

Cette réorganisation inclut des corrections de comportement ; elle ne reproduit pas les défauts du prototype bit pour bit.

- La loi `poids = gain × écart_angulaire` est conservée, puis bornée et normalisée.
- L’ancien test comparait un produit vectoriel **au carré** à un sinus non élevé au carré. Le nouveau test compare directement l’angle à `0.4 × dt`. Le seuil effectif est donc différent et mérite un nouveau réglage.
- Le visualiseur met désormais à jour la mesure d’accélération précédente à chaque échantillon. Le prototype la gardait à sa valeur de calibration.
- La correction analytique est exprimée dans la même convention de repère que le gyroscope et Madgwick ; les signes du sketch ont été alignés en conséquence.
- Le visualiseur démarre à réception de données, sans exiger une calibration préalable. La calibration reste utile pour réduire la dérive du gyroscope.
- Les anciennes fonctions publiques ne sont pas conservées comme alias : adaptez les appels avec le tableau ci-dessus.
