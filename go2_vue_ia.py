"""
"Ce que voit l'IA de marche" du Unitree Go2 (WebRTC / Wi-Fi). Lecture seule.

Le réseau de neurones qui fait marcher le robot reçoit, à chaque instant, un
vecteur de nombres (son "observation") et renvoie 12 angles cibles pour les moteurs.
Sur un Go2 Pro, on n'a PAS accès à ce vecteur interne : ce script le RECONSTRUIT
à partir des données publiées par le robot, sur le modèle utilisé par Unitree dans
ses outils d'entraînement (unitree_rl_gym) :

  - vitesse de rotation du corps (gyroscope)           3 valeurs
  - direction de la gravité vue par le robot           3 valeurs
  - vitesse du corps (avance / côté / rotation)        3 valeurs
  - angle de chaque articulation - angle "debout"     12 valeurs
  - vitesse de chaque articulation (estimée)          12 valeurs

Les "décisions" de l'IA ne sont pas publiées, mais on en voit l'effet :
c'est le mouvement des articulations (dernier panneau).

Prérequis : pip install matplotlib
Lancer    : python go2_vue_ia.py      (fermer la fenêtre pour arrêter)
"""
import asyncio
import math
import os
import threading
import time

import numpy as np
import matplotlib.pyplot as plt

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

ROBOT_IP = "192.168.2.6"
AES_KEY = "70920cdff469a9d6eee2a89a9bf15452"
INTERVALLE_MS = 100

# Ordre des moteurs Unitree : AVD (avant droit), AVG, ARD, ARG ; chacun hanche, cuisse, genou
NOMS = ["AVD hanche", "AVD cuisse", "AVD genou", "AVG hanche", "AVG cuisse", "AVG genou",
        "ARD hanche", "ARD cuisse", "ARD genou", "ARG hanche", "ARG cuisse", "ARG genou"]
# Posture "debout" de référence (valeurs de la config Go2 de unitree_rl_gym)
DEBOUT = np.array([-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5])

etat = {"statut": "Connexion en cours...", "gyro": np.zeros(3), "gravite": np.array([0, 0, -1.0]),
        "vitesse": np.zeros(3), "q": None, "dq": np.zeros(12), "q_prec": None, "t_prec": None}


def gravite_projetee(qw, qx, qy, qz):
    """Direction de la gravité exprimée dans le repère du robot (à plat : [0, 0, -1])."""
    # 3e ligne de la matrice de rotation, transposée -> R^T * [0, 0, -1]
    gx = -(2 * (qx * qz - qw * qy))
    gy = -(2 * (qy * qz + qw * qx))
    gz = -(1 - 2 * (qx * qx + qy * qy))
    return np.array([gx, gy, gz])


def cb_sport(message):
    d = message["data"]
    imu = d["imu_state"]
    etat["gyro"] = np.asarray(imu["gyroscope"], dtype=float)
    etat["gravite"] = gravite_projetee(*imu["quaternion"])          # quaternion = [w, x, y, z]
    v = d.get("velocity", [0, 0, 0])
    etat["vitesse"] = np.array([v[0], v[1], d.get("yaw_speed", 0.0)], dtype=float)


def cb_low(message):
    moteurs = message["data"]["motor_state"][:12]
    q = np.array([m["q"] for m in moteurs], dtype=float)
    t = time.time()
    if etat["q_prec"] is not None and t > etat["t_prec"]:
        brut = (q - etat["q_prec"]) / (t - etat["t_prec"])     # dérivée de l'angle
        etat["dq"] = 0.6 * etat["dq"] + 0.4 * brut              # léger lissage
    etat["q"], etat["q_prec"], etat["t_prec"] = q, q, t


def thread_webrtc():
    async def main():
        conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP, aes_128_key=AES_KEY)
        await conn.connect()
        await conn.datachannel.disableTrafficSaving(True)       # lowstate à ~20 Hz au lieu de 1 Hz
        conn.datachannel.pub_sub.subscribe("rt/lf/sportmodestate", cb_sport)
        conn.datachannel.pub_sub.subscribe("rt/lf/lowstate", cb_low)
        etat["statut"] = "Connecté"
        while True:
            await asyncio.sleep(1)
    try:
        asyncio.run(main())
    except Exception as e:
        etat["statut"] = f"Erreur : {e}"


# ---------------------------------------------------------------- fenêtre

fig = plt.figure("Go2 - ce que voit l'IA de marche", figsize=(14, 8))
gs = fig.add_gridspec(2, 3, height_ratios=[1, 1.6], hspace=0.45, wspace=0.55)


def panneau(position, titre, etiquettes, limite, unite, couleur):
    ax = fig.add_subplot(position)
    y = np.arange(len(etiquettes))
    barres = ax.barh(y, np.zeros(len(etiquettes)), color=couleur)
    ax.set_yticks(y, etiquettes, fontsize=8)
    ax.invert_yaxis()
    ax.set_xlim(-limite, limite)
    ax.axvline(0, color="black", linewidth=0.8)
    ax.set_title(titre, fontsize=10)
    ax.set_xlabel(unite, fontsize=8)
    ax.grid(axis="x", alpha=0.3)
    textes = [ax.text(limite * 0.97, i, "", va="center", ha="right", fontsize=7) for i in y]
    return ax, barres, textes


p_gyro = panneau(gs[0, 0], "1. Rotation du corps (gyroscope)",
                 ["roulis", "tangage", "lacet"], 2.0, "rad/s", "tab:blue")
p_grav = panneau(gs[0, 1], "2. Où est le bas ? (gravité vue du robot)",
                 ["avant/arrière", "gauche/droite", "haut/bas"], 1.1, "fraction de g", "tab:purple")
p_vit = panneau(gs[0, 2], "3. Vitesse du corps",
                ["avance", "côté", "rotation"], 1.0, "m/s  /  rad/s", "tab:green")
p_ang = panneau(gs[1, 0:2], "4. Angle de chaque articulation, par rapport à la posture « debout »",
                NOMS, 1.0, "écart en rad", "tab:orange")
p_dq = panneau(gs[1, 2], "5. Vitesse des articulations\n(= l'effet des décisions de l'IA)",
               NOMS, 3.0, "rad/s", "tab:red")
titre = fig.suptitle("", fontsize=11)

PANNEAUX = [p_gyro, p_grav, p_vit, p_ang, p_dq]
for _, barres, textes in PANNEAUX:
    for a in list(barres) + textes:
        a.set_animated(True)
titre.set_animated(True)
fond = {"img": None}


def remplir(p, valeurs):
    ax, barres, textes = p
    for b, t, v in zip(barres, textes, valeurs):
        b.set_width(v)
        t.set_text(f"{v:+.2f}")


def dessiner():
    for ax, barres, textes in PANNEAUX:
        for a in list(barres) + textes:
            ax.draw_artist(a)
    fig.draw_artist(titre)


def sur_dessin(_):
    fond["img"] = fig.canvas.copy_from_bbox(fig.bbox)
    dessiner()
    fig.canvas.blit(fig.bbox)


def tic():
    remplir(p_gyro, etat["gyro"])
    remplir(p_grav, etat["gravite"])
    remplir(p_vit, etat["vitesse"])
    if etat["q"] is not None:
        remplir(p_ang, etat["q"] - DEBOUT)
        remplir(p_dq, etat["dq"])
    nb = 3 + 3 + 3 + 12 + 12
    titre.set_text(f"{etat['statut']}   |   Reconstruction des {nb} nombres que l'IA reçoit à chaque instant")
    if fond["img"] is None:
        fig.canvas.draw_idle()
        return
    fig.canvas.restore_region(fond["img"])
    dessiner()
    fig.canvas.blit(fig.bbox)
    fig.canvas.flush_events()


if __name__ == "__main__":
    threading.Thread(target=thread_webrtc, daemon=True).start()
    fig.canvas.mpl_connect("draw_event", sur_dessin)
    fig.canvas.mpl_connect("close_event", lambda _: os._exit(0))
    minuterie = fig.canvas.new_timer(interval=INTERVALLE_MS)
    minuterie.add_callback(tic)
    minuterie.start()
    plt.show()
    os._exit(0)
