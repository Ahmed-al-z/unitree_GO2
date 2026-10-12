"""
Affichage détaillé des messages du Unitree Go2 (WebRTC / Wi-Fi). Lecture seule.

Deux façons de l'utiliser :

  1) Rapport complet de tous les topics :
        python go2_detail.py
     -> écoute 10 s, puis affiche CHAQUE champ de CHAQUE topic actif
        et enregistre le tout dans rapport_go2.txt

  2) Un seul topic en direct (rafraîchi en continu) :
        python go2_detail.py UWB_STATE
        python go2_detail.py LF_SPORT_MOD_STATE
     -> Ctrl+C pour arrêter

Fermer l'appli Unitree Go sur le téléphone avant de lancer.
"""
import asyncio
import json
import os
import sys
import time

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

ROBOT_IP = "192.168.2.6"
AES_KEY = "70920cdff469a9d6eee2a89a9bf15452"

DUREE_RAPPORT = 10      # secondes d'écoute pour le rapport complet
FICHIER_RAPPORT = "rapport_go2.txt"
MAX_VALEURS = 12        # au-delà, les longues listes sont raccourcies

TOPICS = {
    "LOW_STATE":               "rt/lf/lowstate",
    "LF_SPORT_MOD_STATE":      "rt/lf/sportmodestate",
    "SPORT_MOD_STATE":         "rt/sportmodestate",
    "ROBOTODOM":               "rt/utlidar/robot_pose",
    "UWB_STATE":               "rt/uwbstate",
    "ULIDAR_STATE":            "rt/utlidar/lidar_state",
    "ULIDAR_ARRAY":            "rt/utlidar/voxel_map_compressed",
    "MULTIPLE_STATE":          "rt/multiplestate",
    "AUDIO_HUB_PLAY_STATE":    "rt/audiohub/player/state",
    "WIRELESS_CONTROLLER":     "rt/wirelesscontroller",
    "SELF_TEST":               "rt/selftest",
    "SERVICE_STATE":           "rt/servicestate",
    "GRID_MAP":                "rt/mapping/grid_map",
    "SLAM_ODOMETRY":           "rt/lio_sam_ros2/mapping/odometry",
    "LIDAR_MAPPING_ODOM":      "rt/uslam/frontend/odom",
    "LIDAR_LOCALIZATION_ODOM": "rt/uslam/localization/odom",
    "GAS_SENSOR":              "rt/gas_sensor",
    "GPT_FEEDBACK":            "rt/gptflowfeedback",
}
LIDAR_TOPICS = {"ULIDAR_ARRAY", "ULIDAR_STATE"}


# ---------------------------------------------------------------- mise en forme

def formater_valeur(v):
    if isinstance(v, float):
        return f"{v:.4f}"
    if hasattr(v, "tolist"):                      # tableau numpy
        v = v.tolist()
    if isinstance(v, (bytes, bytearray)):
        return f"<{len(v)} octets binaires>"
    if isinstance(v, (list, tuple)):
        if len(v) > MAX_VALEURS:
            debut = ", ".join(formater_valeur(x) for x in v[:MAX_VALEURS])
            return f"[{debut}, ...]  ({len(v)} valeurs au total)"
        return "[" + ", ".join(formater_valeur(x) for x in v) + "]"
    return str(v)


def aplatir(obj, prefixe=""):
    """Transforme un message imbriqué en lignes 'chemin.du.champ = valeur'."""
    lignes = []
    # Certains topics envoient du JSON sous forme de texte : on le décode.
    if isinstance(obj, str):
        try:
            obj = json.loads(obj)
        except (ValueError, TypeError):
            return [(prefixe or "(texte)", obj)]
    if hasattr(obj, "tolist") and not isinstance(obj, (str, bytes)):
        obj = obj.tolist()

    if isinstance(obj, dict):
        for k, v in obj.items():
            lignes += aplatir(v, f"{prefixe}.{k}" if prefixe else str(k))
    elif isinstance(obj, (list, tuple)) and obj and all(isinstance(x, dict) for x in obj):
        for i, x in enumerate(obj):               # ex : motor_state[0], motor_state[1]...
            lignes += aplatir(x, f"{prefixe}[{i}]")
    else:
        lignes.append((prefixe or "(valeur)", formater_valeur(obj)))
    return lignes


def bloc_texte(nom, topic, message, nb=None, freq=None):
    entete = f"■ {nom}   ({topic})"
    if nb is not None:
        entete += f"   —   {nb} messages, {freq:.1f} Hz"
    lignes = aplatir(message)
    largeur = min(max((len(c) for c, _ in lignes), default=10), 45)
    corps = "\n".join(f"    {c:<{largeur}} = {v}" for c, v in lignes)
    return f"{entete}\n{'-' * len(entete)}\n{corps}\n"


def extraire(message):
    return message.get("data", message) if isinstance(message, dict) else message


# ---------------------------------------------------------------- connexion

async def connecter(lidar):
    print(f"Connexion au robot {ROBOT_IP}...")
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP, aes_128_key=AES_KEY)
    await conn.connect()
    print("Connecté !")
    if lidar:
        try:
            await conn.datachannel.disableTrafficSaving(True)
            conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")
        except Exception as e:
            print(f"(LiDAR non activé : {e})")
    return conn


# ---------------------------------------------------------------- mode 1 : rapport

async def rapport():
    conn = await connecter(lidar=True)
    stats = {n: {"count": 0, "first": 0.0, "last": 0.0, "msg": None} for n in TOPICS}

    def make_cb(nom):
        def cb(message):
            s = stats[nom]
            t = time.time()
            if s["count"] == 0:
                s["first"] = t
            s["count"] += 1
            s["last"] = t
            s["msg"] = extraire(message)          # on garde le message le plus récent
        return cb

    for nom, topic in TOPICS.items():
        conn.datachannel.pub_sub.subscribe(topic, make_cb(nom))

    for restant in range(DUREE_RAPPORT, 0, -1):
        print(f"\r  Écoute... {restant:2d} s", end="", flush=True)
        await asyncio.sleep(1)
    print("\n")

    blocs = []
    for nom, s in sorted(stats.items(), key=lambda kv: -kv[1]["count"]):
        if s["count"] == 0:
            continue
        duree = s["last"] - s["first"]
        freq = (s["count"] - 1) / duree if duree > 0 else 0.0
        blocs.append(bloc_texte(nom, TOPICS[nom], s["msg"], s["count"], freq))

    muets = [n for n, s in stats.items() if s["count"] == 0]
    texte = "\n".join(blocs) + "\nTopics sans message : " + (", ".join(muets) or "(aucun)") + "\n"

    print(texte)
    with open(FICHIER_RAPPORT, "w", encoding="utf-8") as f:
        f.write(texte)
    print(f"Rapport enregistré dans : {os.path.abspath(FICHIER_RAPPORT)}")


# ---------------------------------------------------------------- mode 2 : direct

async def direct(nom):
    if nom not in TOPICS:
        print(f"Topic inconnu : {nom}\nChoix possibles : {', '.join(TOPICS)}")
        return
    conn = await connecter(lidar=nom in LIDAR_TOPICS)
    etat = {"count": 0, "dernier_affichage": 0.0}

    def cb(message):
        etat["count"] += 1
        t = time.time()
        if t - etat["dernier_affichage"] < 0.2:   # 5 rafraîchissements/s max, lisible
            return
        etat["dernier_affichage"] = t
        sys.stdout.write("\033[H\033[J")
        print(bloc_texte(nom, TOPICS[nom], extraire(message)))
        print(f"Messages reçus : {etat['count']}      (Ctrl+C pour arrêter)")
        sys.stdout.flush()

    conn.datachannel.pub_sub.subscribe(TOPICS[nom], cb)
    print(f"En attente de messages sur {nom}...")
    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    try:
        if len(sys.argv) > 1:
            asyncio.run(direct(sys.argv[1].upper()))
        else:
            asyncio.run(rapport())
    except KeyboardInterrupt:
        print("\nArrêt.")
    os._exit(0)
