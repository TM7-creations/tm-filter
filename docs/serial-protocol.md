# Protocole série du démonstrateur

Chaque trame contient 30 octets, envoyés à 115200 bauds :

| Décalage | Taille | Type little-endian | Champ |
|---:|---:|---|---|
| 0 | 2 | octets | synchronisation `AA 55` |
| 2 | 12 | 3 × IEEE-754 float32 | `ax, ay, az` en m/s² |
| 14 | 12 | 3 × IEEE-754 float32 | `gx, gy, gz` en deg/s |
| 26 | 4 | int32 | compteur d’échantillons |

La valeur de compteur `-1` marque le démarrage du capteur ou la fin de calibration et demande la remise à zéro des filtres. Le compteur repart ensuite à zéro. Le visualiseur détecte les échantillons manquants et calcule `dt = nombre_de_pas / fréquence_nominale`.

Le décodeur cherche l’en-tête dans un flux arbitrairement découpé et rejette les nombres non finis. Comme le protocole n’a ni longueur/version explicite ni somme de contrôle, une corruption contenant par hasard des valeurs finies peut rester indétectable. Pour un usage autre que la démonstration locale, ajoutez au minimum une version, une longueur et un CRC.

La fréquence est nominale. Le sketch n’envoie pas d’horodatage : le compteur détecte les pertes de trames transmises, mais pas les retards de boucle ou les lectures I²C ratées. Pour étudier la précision en dynamique, ajoutez un horodatage capteur à chaque trame.

Le visualiseur refuse une extrapolation de plus de 0.5 seconde sur une seule mesure après une coupure. Pendant cet intervalle, l’orientation ne peut pas être reconstruite à partir des données manquantes.
