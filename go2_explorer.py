"""
Explorateur de données du Unitree Go2 (WebRTC / Wi-Fi).

S'abonne à TOUS les topics d'état connus, écoute pendant DUREE secondes, puis :
  - affiche quels topics publient, combien de messages et à quelle fréquence ;
  - enregistre un exemple de message par topic dans le dossier exploration_go2/
    (un fichier .json par topic), pour que tu puisses explorer leur structure.

Le script est en LECTURE SEULE : il n'envoie aucune commande de mouvement.
La seule chose qu'il envoie (si ACTIVER_LIDAR = True) est l'allumage du flux LiDAR.

Fermer l'appli Unitree Go sur le téléphone avant de lancer.
"""
import asyncio
import json
import os
import time

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

ROBOT_IP = "192.168.2.6"
AES_KEY = "70920cdff469a9d6eee2a89a9bf15452"

DUREE = 20              # secondes d'écoute
ACTIVER_LIDAR = True    # allume le flux du LiDAR (beaucoup de données)
DOSSIER = "exploration_go2"

# Topics d'état (lecture). Les topics "request" / "cmd" sont volontairement exclus.
TOPICS = {
    "LOW_STATE":               "rt/lf/lowstate",
    "SPORT_MOD_STATE":         "rt/sportmodestate",
    "LF_SPORT_MOD_STATE":      "rt/lf/sportmodestate",
    "MULTIPLE_STATE":          "rt/multiplestate",
    "WIRELESS_CONTROLLER":     "rt/wirelesscontroller",
    "UWB_STATE":               "rt/uwbstate",
    "ULIDAR_STATE":            "rt/utlidar/lidar_state",
    "ULIDAR_ARRAY":            "rt/utlidar/voxel_map_compressed",
    "ULIDAR":                  "rt/utlidar/voxel_map",
    "ROBOTODOM":               "rt/utlidar/robot_pose",
    "SELF_TEST":               "rt/selftest",
    "SERVICE_STATE":           "rt/servicestate",
    "GRID_MAP":                "rt/mapping/grid_map",
    "SLAM_ODOMETRY":           "rt/lio_sam_ros2/mapping/odometry",
    "LIDAR_MAPPING_ODOM":      "rt/uslam/frontend/odom",
    "LIDAR_LOCALIZATION_ODOM": "rt/uslam/localization/odom",
    "AUDIO_HUB_PLAY_STATE":    "rt/audiohub/player/state",
    "GAS_SENSOR":              "rt/gas_sensor",
    "GPT_FEEDBACK":            "rt/gptflowfeedback",
}

stats = {name: {"count": 0, "first": None, "last": None, "example": None} for name in TOPICS}


def reduire(obj, max_list=30):
    """Rend un message lisible et sérialisable en JSON (tronque les gros tableaux)."""
    if isinstance(obj, dict):
        return {str(k): reduire(v, max_list) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        if len(obj) > max_list:
            return [reduire(v, max_list) for v in obj[:max_list]] + [f"... ({len(obj)} éléments au total)"]
        return [reduire(v, max_list) for v in obj]
    if isinstance(obj, (bytes, bytearray)):
        return f"<{len(obj)} octets binaires>"
    if hasattr(obj, "tolist"):          # tableaux numpy
        return reduire(obj.tolist(), max_list)
    if isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj
    return str(obj)


def make_callback(name):
    def callback(message):
        s = stats[name]
        now = time.time()
        if s["count"] == 0:
            s["first"] = now
            s["example"] = reduire(message.get("data", message) if isinstance(message, dict) else message)
        s["count"] += 1
        s["last"] = now
    return callback


def sauvegarder():
    os.makedirs(DOSSIER, exist_ok=True)
    for name, s in stats.items():
        if s["example"] is not None:
            chemin = os.path.join(DOSSIER, f"{name}.json")
            with open(chemin, "w", encoding="utf-8") as f:
                json.dump(s["example"], f, indent=2, ensure_ascii=False)


def afficher_resume():
    print("\n" + "=" * 66)
    print(f"{'TOPIC':<26}{'MESSAGES':>10}{'FRÉQ. (Hz)':>14}   CHAMPS")
    print("=" * 66)
    for name, s in sorted(stats.items(), key=lambda kv: -kv[1]["count"]):
        if s["count"] == 0:
            continue
        duree = (s["last"] - s["first"]) if s["count"] > 1 else 0
        freq = (s["count"] - 1) / duree if duree > 0 else 0
        champs = list(s["example"].keys()) if isinstance(s["example"], dict) else type(s["example"]).__name__
        print(f"{name:<26}{s['count']:>10}{freq:>14.1f}   {champs}")
    muets = [n for n, s in stats.items() if s["count"] == 0]
    print("-" * 66)
    print("Aucun message reçu sur :", ", ".join(muets) if muets else "(aucun)")
    print(f"\nExemples de messages enregistrés dans le dossier : {os.path.abspath(DOSSIER)}")


async def main():
    print(f"Connexion au robot {ROBOT_IP}...")
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP, aes_128_key=AES_KEY)
    await conn.connect()
    print("Connecté !")

    if ACTIVER_LIDAR:
        try:
            await conn.datachannel.disableTrafficSaving(True)
            conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")
            print("Flux LiDAR activé.")
        except Exception as e:
            print(f"(Impossible d'activer le LiDAR : {e})")

    for name, topic in TOPICS.items():
        try:
            conn.datachannel.pub_sub.subscribe(topic, make_callback(name))
        except Exception as e:
            print(f"Abonnement impossible à {topic} : {e}")

    print(f"Écoute de {len(TOPICS)} topics pendant {DUREE} s...")
    for restant in range(DUREE, 0, -1):
        actifs = sum(1 for s in stats.values() if s["count"] > 0)
        print(f"\r  {restant:2d} s restantes — {actifs} topic(s) actif(s)   ", end="", flush=True)
        await asyncio.sleep(1)

    sauvegarder()
    afficher_resume()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nInterrompu.")
        sauvegarder()
        afficher_resume()
    os._exit(0)
