"""
Marche autonome du Unitree Go2 (WebRTC / Wi-Fi).

Le robot fait un "carré" : il avance de DISTANCE_SEGMENT mètres, puis tourne de 90°
à gauche, et recommence NB_SEGMENTS fois. Il utilise son LiDAR pour :
  - s'arrêter s'il voit un obstacle devant lui (plus près que DIST_ARRET) ;
  - ne tourner que s'il y a assez de place autour de lui.
Et son odométrie (position estimée) pour mesurer la distance parcourue et l'angle tourné.

DEUX MODES :
  python go2_autonome.py          -> MODE TEST : le robot NE BOUGE PAS. Le script affiche
                                     en direct ce que le robot "voit" (obstacle devant,
                                     place pour tourner). À faire en premier !
  python go2_autonome.py --vrai   -> le robot se lève et marche tout seul.

SÉCURITÉ :
  - pièce dégagée, personne sur le trajet, télécommande en main ;
  - Ctrl+C arrête le robot immédiatement ;
  - arrêt automatique si les capteurs ne répondent plus ou après DUREE_MAX secondes.
"""
import asyncio
import json
import math
import os
import random
import sys
import time

import numpy as np

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

ROBOT_IP = "192.168.2.6"
AES_KEY = "70920cdff469a9d6eee2a89a9bf15452"

# ----------------------------------------------------------------- réglages
DISTANCE_SEGMENT = 2.0    # m à parcourir tout droit avant de tourner
NB_SEGMENTS = 4           # 4 côtés = un carré
ANGLE_VIRAGE = 90         # degrés, vers la gauche
VITESSE = 0.3             # m/s (lent)
VITESSE_ROT = 0.5         # rad/s (lent)

DIST_ARRET = 0.85         # m : s'arrêter si un obstacle est plus près que ça devant (depuis le centre du robot)
DEMI_LARGEUR = 0.35       # m : largeur du "couloir" surveillé devant le robot (de chaque côté)
RAYON_VIRAGE = 0.7        # m : place libre nécessaire autour du robot pour tourner sur place
H_MIN, H_MAX = 0.20, 0.70 # m : hauteurs au-dessus du sol considérées comme obstacle
                          #     (le sol vu par le LiDAR a ~15 cm d'"épaisseur" : en dessous de 20 cm = sol)
X_MIN_DEVANT = 0.40       # m : on ne surveille qu'au-delà de l'avant du corps
ZONE_CORPS = (-0.50, 0.40, 0.30)  # (arrière, avant, demi-largeur) en m : zone du robot lui-même, ignorée
MIN_POINTS = 3            # nb de points LiDAR minimum pour croire à un obstacle (anti-parasites)

DUREE_MAX = 120           # s : arrêt de sécurité global
TIMEOUT_CAPTEURS = 1.5    # s : arrêt si plus de données capteurs depuis ce temps

CMD = {"BalanceStand": 1002, "StopMove": 1003, "StandUp": 1004, "Move": 1008}

capteurs = {"pose": None, "t_pose": 0.0, "pts": None, "t_lidar": 0.0}


class ArretSecurite(Exception):
    pass


# ----------------------------------------------------------------- capteurs

def lacet(o):
    x, y, z, w = o["x"], o["y"], o["z"], o["w"]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def cb_pose(message):
    p = message["data"]["pose"]
    capteurs["pose"] = (p["position"]["x"], p["position"]["y"], lacet(p["orientation"]))
    capteurs["t_pose"] = time.time()


def cb_lidar(message):
    d = message["data"]
    pos = np.asarray(d["data"]["positions"], dtype=np.float32).reshape(-1, 3)
    if len(pos) == 0:
        return
    p = pos.astype(np.int32)
    codes = np.unique((p[:, 0] << 16) | (p[:, 1] << 8) | p[:, 2])
    idx = np.stack([codes >> 16, (codes >> 8) & 255, codes & 255], axis=1).astype(np.float32)
    capteurs["pts"] = idx * float(d["resolution"]) + np.asarray(d["origin"], dtype=np.float32)
    capteurs["t_lidar"] = time.time()


def verifier_capteurs():
    maintenant = time.time()
    if capteurs["pose"] is None or maintenant - capteurs["t_pose"] > TIMEOUT_CAPTEURS:
        raise ArretSecurite("plus de position (odométrie) reçue")
    if capteurs["pts"] is None or maintenant - capteurs["t_lidar"] > TIMEOUT_CAPTEURS:
        raise ArretSecurite("plus de données LiDAR reçues")


def hauteur_sol(z):
    """Hauteur du sol = la couche horizontale la plus peuplée (le sol est la plus grande surface vue)."""
    bins = np.round(z / 0.05).astype(int)
    valeurs, comptes = np.unique(bins, return_counts=True)
    return float(valeurs[np.argmax(comptes)]) * 0.05


def obstacles_repere_robot():
    """Obstacles dans le repère du robot : x vers l'avant, y vers la gauche (en m)."""
    pts, pose = capteurs["pts"], capteurs["pose"]
    if pts is None or pose is None:
        return np.empty(0), np.empty(0), np.empty(0)
    rx, ry, cap = pose
    dx, dy = pts[:, 0] - rx, pts[:, 1] - ry
    c, s = math.cos(cap), math.sin(cap)
    xr = c * dx + s * dy
    yr = -s * dx + c * dy
    proche = (np.abs(xr) < 2.5) & (np.abs(yr) < 2.5)
    if not proche.any():
        return np.empty(0), np.empty(0), np.empty(0)
    h = pts[:, 2] - hauteur_sol(pts[proche, 2])
    garde = proche & (h > H_MIN) & (h < H_MAX)
    arriere, avant, demi = ZONE_CORPS
    corps = (xr > arriere) & (xr < avant) & (np.abs(yr) < demi)   # ignorer le robot lui-même
    garde &= ~corps
    return xr[garde], yr[garde], h[garde]


def obstacle_devant():
    """Distance (m) du plus proche obstacle dans le couloir devant le robot (inf si rien)."""
    xr, yr, _ = obstacles_repere_robot()
    sel = (xr > X_MIN_DEVANT) & (np.abs(yr) < DEMI_LARGEUR)
    if sel.sum() < MIN_POINTS:
        return math.inf
    return float(np.sort(xr[sel])[MIN_POINTS - 1])


def place_pour_tourner():
    xr, yr, _ = obstacles_repere_robot()
    return int((np.hypot(xr, yr) < RAYON_VIRAGE).sum()) < MIN_POINTS


def points_proches(n=5):
    """Les n points d'obstacle les plus proches (pour comprendre ce que le robot voit)."""
    xr, yr, h = obstacles_repere_robot()
    if len(xr) == 0:
        return "aucun"
    ordre = np.argsort(np.hypot(xr, yr))[:n]
    return "  ".join(f"(av {xr[i]:+.2f}, g {yr[i]:+.2f}, h {h[i]:.2f})" for i in ordre)


def points_devant(n=3):
    """Les n points les plus proches DANS le couloir devant le robot."""
    xr, yr, h = obstacles_repere_robot()
    sel = np.where((xr > X_MIN_DEVANT) & (np.abs(yr) < DEMI_LARGEUR))[0]
    if len(sel) == 0:
        return "aucun"
    ordre = sel[np.argsort(xr[sel])][:n]
    return "  ".join(f"(av {xr[i]:+.2f}, g {yr[i]:+.2f}, h {h[i]:.2f})" for i in ordre)


def ecart_angle(a, b):
    """a - b ramené entre -pi et +pi."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


# ----------------------------------------------------------------- commandes

async def appeler(conn, api_id):
    rep = await conn.datachannel.pub_sub.publish_request_new("rt/api/sport/request", {"api_id": api_id})
    return rep.get("data", {}).get("header", {}).get("status", {}).get("code", -1)


def envoyer(conn, api_id, parametre=None):
    ident = int(time.time() * 1000) % 2147483648 + random.randint(0, 1000)
    conn.datachannel.pub_sub.publish_without_callback("rt/api/sport/request", {
        "header": {"identity": {"id": ident, "api_id": api_id},
                   "policy": {"priority": 0, "noreply": True}},
        "parameter": json.dumps(parametre) if parametre is not None else "",
        "binary": [],
    })


def bouger(conn, vx, vyaw):
    envoyer(conn, CMD["Move"], {"x": vx, "y": 0, "z": vyaw})


async def stopper(conn):
    for _ in range(3):                      # plusieurs fois, par sécurité
        envoyer(conn, CMD["Move"], {"x": 0, "y": 0, "z": 0})
        envoyer(conn, CMD["StopMove"])
        await asyncio.sleep(0.05)


# ----------------------------------------------------------------- comportements

async def avancer(conn, t_debut):
    x0, y0, _ = capteurs["pose"]
    t_seg = time.time()
    t_affiche = 0.0
    while True:
        if time.time() - t_debut > DUREE_MAX:
            raise ArretSecurite("durée maximale atteinte")
        verifier_capteurs()
        x, y, _ = capteurs["pose"]
        parcouru = math.hypot(x - x0, y - y0)
        devant = obstacle_devant()
        if parcouru >= DISTANCE_SEGMENT:
            return f"distance atteinte ({parcouru:.2f} m)"
        if devant < DIST_ARRET:
            return f"OBSTACLE à {devant:.2f} m devant (après {parcouru:.2f} m)"
        if time.time() - t_seg > DISTANCE_SEGMENT / VITESSE * 2 + 5:
            return f"n'avance plus (bloqué ?) après {parcouru:.2f} m"
        bouger(conn, VITESSE, 0)
        if time.time() - t_affiche > 0.5:
            t_affiche = time.time()
            txt = "rien" if math.isinf(devant) else f"{devant:.2f} m"
            print(f"   avance... {parcouru:4.2f}/{DISTANCE_SEGMENT} m   obstacle devant : {txt}")
        await asyncio.sleep(0.1)


async def tourner(conn, t_debut):
    _, _, cap0 = capteurs["pose"]
    cible = math.radians(ANGLE_VIRAGE)
    t_virage = time.time()
    while True:
        if time.time() - t_debut > DUREE_MAX:
            raise ArretSecurite("durée maximale atteinte")
        verifier_capteurs()
        tourne = abs(ecart_angle(capteurs["pose"][2], cap0))
        if tourne >= cible - math.radians(3):
            return f"virage fait ({math.degrees(tourne):.0f}°)"
        if time.time() - t_virage > cible / VITESSE_ROT * 2 + 5:
            return f"virage incomplet ({math.degrees(tourne):.0f}°)"
        bouger(conn, 0, VITESSE_ROT)
        await asyncio.sleep(0.1)


async def mode_vrai(conn):
    print("\nLe robot se lève...")
    await appeler(conn, CMD["StandUp"])
    await asyncio.sleep(2)
    await appeler(conn, CMD["BalanceStand"])
    await asyncio.sleep(1.5)
    verifier_capteurs()
    print(f"   points d'obstacle les plus proches : {points_proches()}")

    t_debut = time.time()
    try:
        for i in range(1, NB_SEGMENTS + 1):
            print(f"\n--- Segment {i}/{NB_SEGMENTS} : j'avance ---")
            raison = await avancer(conn, t_debut)
            await stopper(conn)
            print(f"   arrêt : {raison}")
            if raison.startswith("OBSTACLE"):
                print(f"   (points vus devant : {points_devant(3)})")
            await asyncio.sleep(0.8)

            if not place_pour_tourner():
                print("   pas assez de place pour tourner sans risque -> je m'arrête là.")
                print(f"   (points trop proches : {points_proches(3)})")
                break
            print(f"   la voie est libre, je tourne de {ANGLE_VIRAGE}° à gauche...")
            print(f"   {await tourner(conn, t_debut)}")
            await stopper(conn)
            await asyncio.sleep(0.8)
        print("\nParcours terminé.")
    except ArretSecurite as e:
        print(f"\nARRÊT DE SÉCURITÉ : {e}")
    finally:
        await stopper(conn)
        print("Robot arrêté (il reste debout).")


async def mode_test(conn):
    print("\nMODE TEST : le robot ne bouge pas. Passe devant lui ou pose un carton pour voir")
    print("s'il le détecte. Ctrl+C pour arrêter.\n")
    while True:
        try:
            verifier_capteurs()
            devant = obstacle_devant()
            txt = "rien dans le couloir" if math.isinf(devant) else f"{devant:.2f} m"
            arret = "  -> je m'ARRÊTERAIS" if devant < DIST_ARRET else ""
            x, y, cap = capteurs["pose"]
            place = "oui" if place_pour_tourner() else "NON"
            print(f"obstacle devant : {txt:<22}{arret:<22} place pour tourner : {place}")
            print(f"   devant (av = devant, g = à gauche, h = hauteur, en m) : {points_devant(3)}")
            print(f"   autour : {points_proches(3)}")
        except ArretSecurite as e:
            print(f"(en attente des capteurs : {e})")
        await asyncio.sleep(0.5)


async def main():
    vrai = "--vrai" in sys.argv
    if vrai:
        print("=" * 64)
        print(" LE ROBOT VA MARCHER TOUT SEUL.")
        print(f" Trajet : {NB_SEGMENTS} x ({DISTANCE_SEGMENT} m tout droit + {ANGLE_VIRAGE}° à gauche)")
        print(" - au moins 3 m x 3 m dégagés, personne sur le trajet")
        print(" - télécommande en main ; Ctrl+C = arrêt immédiat")
        print("=" * 64)
        await asyncio.to_thread(input, "Appuie sur Entrée pour démarrer...")

    print(f"Connexion au robot {ROBOT_IP}...")
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP, aes_128_key=AES_KEY)
    await conn.connect()
    await conn.datachannel.disableTrafficSaving(True)
    conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")
    conn.datachannel.pub_sub.subscribe("rt/utlidar/robot_pose", cb_pose)
    conn.datachannel.pub_sub.subscribe("rt/utlidar/voxel_map_compressed", cb_lidar)
    print("Connecté. Attente des premières données capteurs...")
    await asyncio.sleep(2)

    if vrai:
        try:
            await mode_vrai(conn)
        except asyncio.CancelledError:
            await stopper(conn)
            raise
    else:
        await mode_test(conn)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nArrêt demandé (Ctrl+C).")
    os._exit(0)
