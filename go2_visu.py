"""
Visualisation en direct du Unitree Go2 (WebRTC / Wi-Fi). Lecture seule.

Une fenêtre avec trois vues :
  - gauche  : carte du LiDAR vue de dessus (gris = sol, couleur = obstacles) + position et
              orientation du robot ;
  - haut droite : position de la télécommande vue par l'UWB (ISS 2.0) ;
  - bas droite  : roulis / tangage du corps et hauteur, sur les 10 dernières secondes.

Prérequis (une seule fois) :  pip install matplotlib
Lancer :                      python go2_visu.py
Fermer la fenêtre pour arrêter.
"""
import asyncio
import math
import os
import threading
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

ROBOT_IP = "192.168.2.6"
AES_KEY = "70920cdff469a9d6eee2a89a9bf15452"

RAYON_CARTE = 4.0          # m affichés autour du robot
MAX_OBSTACLES = 6000       # points colorés max (garder l'affichage fluide)
MAX_SOL = 2500             # points de sol (gris) max
SEUIL_OBSTACLE = 0.12      # m au-dessus du sol pour être compté comme obstacle
INTERVALLE_MS = 80         # rafraîchissement de la fenêtre (~12 images/s)
HISTORIQUE_S = 10          # secondes affichées pour l'IMU

etat = {"statut": "Connexion en cours...", "uwb": None, "pose": None, "sol": None, "obstacles": None,
        "nb_lidar": 0, "t_lidar": deque(maxlen=20), "t_uwb": deque(maxlen=20), "t_ecran": deque(maxlen=20)}
historique = deque(maxlen=400)   # (temps, roulis, tangage, hauteur)


# ---------------------------------------------------------------- réception (thread WebRTC)

def cb_uwb(message):
    etat["uwb"] = message["data"]
    etat["t_uwb"].append(time.time())


def cb_pose(message):
    etat["pose"] = message["data"]["pose"]


def cb_sport(message):
    d = message["data"]
    rpy = d["imu_state"]["rpy"]
    historique.append((time.time(), rpy[0], rpy[1], d.get("body_height", 0.0)))


def cb_lidar(message):
    d = message["data"]
    pos = np.asarray(d["data"]["positions"], dtype=np.float32).reshape(-1, 3)
    if len(pos) == 0:
        return
    # positions = indices de voxels -> coordonnées en mètres dans le repère "odom"
    # dédoublonnage rapide : un entier unique par voxel
    p = pos.astype(np.int32)
    codes = np.unique((p[:, 0] << 16) | (p[:, 1] << 8) | p[:, 2])
    idx = np.stack([codes >> 16, (codes >> 8) & 255, codes & 255], axis=1).astype(np.float32)
    pts = idx * float(d["resolution"]) + np.asarray(d["origin"], dtype=np.float32)
    # séparation sol / obstacles : le sol = les points les plus bas
    sol_z = float(np.percentile(pts[:, 2], 10))
    hauteur = pts[:, 2] - sol_z
    etat["sol"] = pts[hauteur < SEUIL_OBSTACLE]
    etat["obstacles"] = np.column_stack([pts[hauteur >= SEUIL_OBSTACLE][:, :2], hauteur[hauteur >= SEUIL_OBSTACLE]])
    etat["nb_lidar"] += 1
    t = time.time()
    etat["t_lidar"].append(t)


def thread_webrtc():
    async def main():
        conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP, aes_128_key=AES_KEY)
        await conn.connect()
        await conn.datachannel.disableTrafficSaving(True)
        conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")
        sub = conn.datachannel.pub_sub.subscribe
        sub("rt/uwbstate", cb_uwb)
        sub("rt/utlidar/robot_pose", cb_pose)
        sub("rt/lf/sportmodestate", cb_sport)
        sub("rt/utlidar/voxel_map_compressed", cb_lidar)
        etat["statut"] = "Connecté"
        while True:
            await asyncio.sleep(1)

    try:
        asyncio.run(main())
    except Exception as e:
        etat["statut"] = f"Erreur : {e}"


# ---------------------------------------------------------------- outils

def lacet_depuis_quaternion(o):
    x, y, z, w = o["x"], o["y"], o["z"], o["w"]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


# ---------------------------------------------------------------- fenêtre

fig = plt.figure("Unitree Go2 - visualisation", figsize=(14, 7.5))
gs = fig.add_gridspec(2, 2, width_ratios=[1.35, 1], hspace=0.35, wspace=0.25)

# Carte LiDAR
ax_carte = fig.add_subplot(gs[:, 0])
ax_carte.set_title("Ce que voit le LiDAR, vu du dessus\ngris = sol   ·   couleur = obstacles (murs, meubles…)")
ax_carte.set_xlabel("x (m)")
ax_carte.set_ylabel("y (m)")
ax_carte.set_aspect("equal")
ax_carte.grid(alpha=0.3)
nuage_sol = ax_carte.scatter(np.empty((0, 2))[:, 0], np.empty((0, 2))[:, 1], s=1, color="0.8")
nuage = ax_carte.scatter([], [], c=[], s=3, cmap="plasma", vmin=0.1, vmax=1.0)
fig.colorbar(nuage, ax=ax_carte, fraction=0.04, pad=0.02, label="hauteur au-dessus du sol (m)")
robot_point, = ax_carte.plot([], [], "o", color="red", markersize=10, label="robot")
robot_dir, = ax_carte.plot([], [], "-", color="red", linewidth=3)
ax_carte.legend(loc="upper right")

# UWB (polaire)
ax_uwb = fig.add_subplot(gs[0, 1], projection="polar")
ax_uwb.set_title("Où est la télécommande ? (UWB / ISS 2.0)", pad=14)
ax_uwb.set_theta_zero_location("N")
ax_uwb.set_thetagrids([0, 45, 90, 135, 180, 225, 270, 315],
                      ["AVANT", "", "GAUCHE", "", "ARRIÈRE", "", "DROITE", ""])
ax_uwb.plot([0], [0], marker=(3, 0, 0), color="red", markersize=14)   # le robot, au centre
ax_uwb.set_autoscale_on(False)
ax_uwb.set_ylim(0, 4)
ax_uwb.set_rlabel_position(300)
tele_point, = ax_uwb.plot([], [], "o", color="orange", markersize=12)
tele_ligne, = ax_uwb.plot([], [], "--", color="orange", alpha=0.6)
tele_texte = ax_uwb.text(0.5, -0.18, "", transform=ax_uwb.transAxes, ha="center")

# IMU
ax_imu = fig.add_subplot(gs[1, 1])
ax_imu.set_title(f"Corps du robot ({HISTORIQUE_S} dernières s)")
ax_imu.set_xlim(-HISTORIQUE_S, 0)
ax_imu.set_ylim(-20, 20)
ax_imu.set_xlabel("temps (s)")
ax_imu.set_ylabel("angle (°)")
ax_imu.grid(alpha=0.3)
l_roulis, = ax_imu.plot([], [], label="roulis")
l_tangage, = ax_imu.plot([], [], label="tangage")
ax_haut = ax_imu.twinx()
ax_haut.set_ylim(0, 0.5)
ax_haut.set_ylabel("hauteur (m)")
l_hauteur, = ax_haut.plot([], [], color="gray", linestyle=":", label="hauteur")
ax_imu.legend(handles=[l_roulis, l_tangage, l_hauteur], loc="upper left", fontsize=8)

titre = fig.suptitle("", fontsize=11, x=0.3, y=0.99)


# Affichage rapide : on ne redessine que ce qui bouge (« blitting »).
# Le fond (axes, grilles, légendes) n'est redessiné que si le cadrage change.
ANIMES = [(ax_carte, nuage_sol), (ax_carte, nuage), (ax_carte, robot_point), (ax_carte, robot_dir),
          (ax_uwb, tele_point), (ax_uwb, tele_ligne), (ax_uwb, tele_texte),
          (ax_imu, l_roulis), (ax_imu, l_tangage), (ax_haut, l_hauteur), (fig, titre)]
for _, artiste in ANIMES:
    artiste.set_animated(True)

vue = {"cx": None, "cy": None, "rmax": 4.0, "fond": None}


def dessiner_animes():
    for conteneur, artiste in ANIMES:
        conteneur.draw_artist(artiste)


def sur_dessin_complet(_event):
    vue["fond"] = fig.canvas.copy_from_bbox(fig.bbox)
    dessiner_animes()
    fig.canvas.blit(fig.bbox)


def mettre_a_jour():
    """Met à jour les données des artistes. Renvoie True si le fond doit être redessiné."""
    fond_a_refaire = False

    pose = etat["pose"]
    rx = ry = 0.0
    if pose:
        rx, ry = pose["position"]["x"], pose["position"]["y"]
        lacet = lacet_depuis_quaternion(pose["orientation"])
        robot_point.set_data([rx], [ry])
        robot_dir.set_data([rx, rx + 0.5 * math.cos(lacet)], [ry, ry + 0.5 * math.sin(lacet)])

    # On ne recadre la carte que si le robot s'éloigne du centre (évite de tout redessiner)
    if vue["cx"] is None or abs(rx - vue["cx"]) > RAYON_CARTE / 2 or abs(ry - vue["cy"]) > RAYON_CARTE / 2:
        vue["cx"], vue["cy"] = rx, ry
        ax_carte.set_xlim(rx - RAYON_CARTE, rx + RAYON_CARTE)
        ax_carte.set_ylim(ry - RAYON_CARTE, ry + RAYON_CARTE)
        fond_a_refaire = True
    cx, cy = vue["cx"], vue["cy"]

    def autour(pts, maxi):
        p = pts[(np.abs(pts[:, 0] - cx) < RAYON_CARTE) & (np.abs(pts[:, 1] - cy) < RAYON_CARTE)]
        return p[:: max(1, len(p) // maxi)]

    if etat["sol"] is not None and len(etat["sol"]):
        nuage_sol.set_offsets(autour(etat["sol"], MAX_SOL)[:, :2])
    if etat["obstacles"] is not None and len(etat["obstacles"]):
        obs = autour(etat["obstacles"], MAX_OBSTACLES)
        nuage.set_offsets(obs[:, :2])
        nuage.set_array(obs[:, 2])

    uwb = etat["uwb"]
    if uwb:
        angle, dist = uwb["yaw_est"], uwb["distance_est"]
        tele_point.set_data([angle], [dist])
        tele_ligne.set_data([angle, angle], [0, dist])
        if dist > vue["rmax"] - 0.3:                     # on agrandit seulement si besoin
            vue["rmax"] = math.ceil(dist + 1)
            ax_uwb.set_ylim(0, vue["rmax"])
            fond_a_refaire = True
        cote = "devant" if abs(angle) < 0.35 else ("à gauche" if angle > 0 else "à droite")
        if abs(angle) > 2.8:
            cote = "derrière"
        tele_texte.set_text(f"à {dist:.2f} m, {cote} du robot ({math.degrees(angle):+.0f}°)")

    if historique:
        maintenant = time.time()
        h = [e for e in list(historique) if maintenant - e[0] <= HISTORIQUE_S]
        if h:
            t = [e[0] - maintenant for e in h]
            l_roulis.set_data(t, [math.degrees(e[1]) for e in h])
            l_tangage.set_data(t, [math.degrees(e[2]) for e in h])
            l_hauteur.set_data(t, [e[3] for e in h])

    def hz(ts):
        ts = list(ts)
        return (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 and ts[-1] > ts[0] else 0.0

    etat["t_ecran"].append(time.time())
    titre.set_text(f"{etat['statut']}   |   LiDAR : {hz(etat['t_lidar']):.1f} cartes/s   "
                   f"|   UWB : {hz(etat['t_uwb']):.1f} mesures/s   |   écran : {hz(etat['t_ecran']):.0f} images/s")
    return fond_a_refaire


def tic():
    canvas = fig.canvas
    if mettre_a_jour() or vue["fond"] is None:
        canvas.draw_idle()            # redessin complet (rare)
        return
    canvas.restore_region(vue["fond"])
    dessiner_animes()
    canvas.blit(fig.bbox)
    canvas.flush_events()


if __name__ == "__main__":
    threading.Thread(target=thread_webrtc, daemon=True).start()
    fig.canvas.mpl_connect("draw_event", sur_dessin_complet)
    fig.canvas.mpl_connect("close_event", lambda _: os._exit(0))
    minuterie = fig.canvas.new_timer(interval=INTERVALLE_MS)
    minuterie.add_callback(tic)
    minuterie.start()
    plt.show()
    os._exit(0)
