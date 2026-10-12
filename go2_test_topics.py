"""
Teste quels topics DDS du Go2 sont transmis par WebRTC (Wi-Fi). LECTURE SEULE.

S'abonne à tous les topics d'état DDS connus du Go2 pendant DUREE secondes et classe :
  - EXPOSÉ  : des messages arrivent par WebRTC
  - MUET    : aucun message (soit non transmis par le pont WebRTC,
              soit la fonction n'est pas active en ce moment)
Les topics de COMMANDE (lowcmd, api/.../request...) ne sont PAS testés : on n'envoie rien.

Lancer :  python go2_test_topics.py
Résultat aussi enregistré dans topics_webrtc.txt
"""
import asyncio
import os
import time

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

ROBOT_IP = "192.168.2.6"
AES_KEY = "70920cdff469a9d6eee2a89a9bf15452"
DUREE = 15

TOPICS = {
    # état bas niveau / mouvement
    "rt/lowstate":                      "capteurs bas niveau, pleine vitesse (~500 Hz en DDS)",
    "rt/lf/lowstate":                   "capteurs bas niveau, basse fréquence",
    "rt/sportmodestate":                "état du mouvement, pleine vitesse",
    "rt/lf/sportmodestate":             "état du mouvement, basse fréquence",
    "rt/wirelesscontroller":            "joysticks et boutons de la télécommande",
    # LiDAR
    "rt/utlidar/cloud":                 "nuage de points BRUT du LiDAR",
    "rt/utlidar/cloud_deskewed":        "nuage de points corrigé du mouvement",
    "rt/utlidar/imu":                   "IMU interne du LiDAR",
    "rt/utlidar/voxel_map":             "carte voxel (non compressée)",
    "rt/utlidar/voxel_map_compressed":  "carte voxel compressée",
    "rt/utlidar/height_map":            "carte des hauteurs du terrain",
    "rt/utlidar/height_map_array":      "carte des hauteurs (tableau)",
    "rt/utlidar/range_info":            "distances aux obstacles",
    "rt/utlidar/lidar_state":           "santé du LiDAR",
    "rt/utlidar/robot_pose":            "position du robot (odométrie LiDAR)",
    "rt/utlidar/robot_odom":            "odométrie complète",
    # UWB / suivi
    "rt/uwbstate":                      "UWB : position de la télécommande (ISS 2.0)",
    # système
    "rt/servicestate":                  "services logiciels actifs",
    "rt/selftest":                      "autodiagnostic",
    "rt/multiplestate":                 "réglages (volume, luminosité, évitement...)",
    "rt/audiohub/player/state":         "état du lecteur audio",
    "rt/gptflowfeedback":               "réponses de l'assistant vocal",
    "rt/gas_sensor":                    "capteur de gaz (accessoire)",
    # cartographie / navigation
    "rt/mapping/grid_map":              "carte 2D (cartographie)",
    "rt/lio_sam_ros2/mapping/odometry": "odométrie SLAM (LIO-SAM)",
    "rt/uslam/frontend/odom":           "odométrie SLAM (uslam)",
    "rt/uslam/frontend/cloud_world_ds": "nuage de la carte SLAM",
    "rt/uslam/localization/odom":       "localisation sur carte",
    "rt/uslam/navigation/global_path":  "chemin de navigation",
    "rt/pctoimage_local":               "image locale de la carte",
    # accessoires
    "rt/arm_Feedback":                  "bras robotique (accessoire)",
}

compte = {t: 0 for t in TOPICS}


def make_cb(t):
    def cb(_):
        compte[t] += 1
    return cb


async def main():
    print(f"Connexion au robot {ROBOT_IP}...")
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP, aes_128_key=AES_KEY)
    await conn.connect()
    await conn.datachannel.disableTrafficSaving(True)
    conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")
    for t in TOPICS:
        conn.datachannel.pub_sub.subscribe(t, make_cb(t))
    for r in range(DUREE, 0, -1):
        print(f"\r  écoute de {len(TOPICS)} topics... {r:2d} s", end="", flush=True)
        await asyncio.sleep(1)

    lignes = ["", f"{'TOPIC DDS':<36}{'RÉSULTAT':<16}{'Hz':>6}   DESCRIPTION", "-" * 100]
    for t, desc in sorted(TOPICS.items(), key=lambda kv: -compte[kv[0]]):
        n = compte[t]
        etat = "EXPOSÉ" if n else "muet"
        lignes.append(f"{t:<36}{etat:<16}{n / DUREE:>6.1f}   {desc}")
    texte = "\n".join(lignes)
    print(texte)
    with open("topics_webrtc.txt", "w", encoding="utf-8") as f:
        f.write(texte + "\n")
    print(f"\nRésultat enregistré dans {os.path.abspath('topics_webrtc.txt')}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nArrêt.")
    os._exit(0)
