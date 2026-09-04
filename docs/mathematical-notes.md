# Notes mathématiques

## Intégration gyroscopique

Avec `q = [w,x,y,z]`, la convention cinématique du projet est

$$
\dot q=\tfrac12q\otimes[0,\boldsymbol\omega]=\tfrac12\Omega q,
$$

où

$$
\Omega=\begin{bmatrix}
0&-\omega_x&-\omega_y&-\omega_z\\
\omega_x&0&\omega_z&-\omega_y\\
\omega_y&-\omega_z&0&\omega_x\\
\omega_z&\omega_y&-\omega_x&0
\end{bmatrix}.
$$

La matrice vérifie `Ω² = -||ω||² I`. En considérant la vitesse angulaire constante entre deux échantillons, `Ω` est fixe sur cet intervalle et la série exponentielle se sépare en termes pairs et impairs :

$$
\exp(\tfrac12\Omega\Delta t)
=I\cos(x)+\frac{\Omega}{\|\boldsymbol\omega\|}\sin(x),
\quad x=\frac{\|\boldsymbol\omega\|\Delta t}{2}.
$$

Pour une vitesse nulle, la formule écrite sous forme de quotient contient une indétermination `0/0`, alors que sa limite est bien définie :

$$
\lim_{\|\boldsymbol\omega\|\to0}
\frac{\sin(\|\boldsymbol\omega\|\Delta t/2)}{\|\boldsymbol\omega\|}
=\frac{\Delta t}{2}.
$$

Le code traite ce cas et utilise une série lorsque le demi-angle est très petit pour évaluer le quotient de façon stable. Il s’agit d’un détail de calcul numérique : la méthode n’est pas limitée aux faibles vitesses. À vitesse exactement nulle, le quaternion ne change pas.

Euler normalisé produit, en posant `u = Ωq/||ω||` :

$$
\frac{q+xu}{\sqrt{1+x^2}}
=q\cos(\arctan x)+u\sin(\arctan x).
$$

Il correspond ainsi à un demi-angle `atan(x)` plutôt qu’à `x`.

## Projection sur la contrainte accélérométrique

Soit `a` le vecteur spécifique normalisé mesuré dans le repère corps. Une orientation compatible vérifie

$$
R(q_a)^T e_z=a.
$$

On construit un quaternion unitaire `b` qui envoie `a` sur `e_z`. La rotation libre autour de `e_z` forme le grand cercle

$$
q_a(t)=b\cos(t)+(e_z\otimes b)\sin(t).
$$

Le quaternion de ce cercle maximisant la valeur absolue du produit scalaire avec la prédiction `q` est sa projection normalisée sur le plan engendré par `b` et `e_z ⊗ b`. Maximiser ce produit scalaire minimise la distance géodésique

$$
d(q,q_a)=2\arccos(|\langle q,q_a\rangle|).
$$

Le code construit `b` avec deux expressions selon l’hémisphère pour éviter la perte de précision près du pôle sud. Si la projection est nulle, toutes les solutions admissibles sont à la même distance et une solution déterministe est retournée.
