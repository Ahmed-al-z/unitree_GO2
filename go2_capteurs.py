"""
Lecture des capteurs du Unitree Go2 Pro via WebRTC (Wi-Fi).
Prérequis : pip install -e . dans le dossier unitree_webrtc_connect
Fermer l'appli Unitree Go sur le téléphone avant de lancer ce script.
"""
import asyncio
import sys

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.constants import RTC_TOPIC

ROBOT_IP = "192.168.2.6"
# Clé AES propre à ton robot (32 caractères hexadécimaux), obtenue avec :
#   python examples\fetch_aes_key.py --email <ton_email_unitree> --password <ton_mdp>
AES_KEY = "70920cdff469a9d6eee2a89a9bf15452"

NOMS_MOTEURS = [
    "AVD hanche", "AVD cuisse", "AVD genou",
    "AVG hanche", "AVG cuisse", "AVG genou",
    "ARD hanche", "ARD cuisse", "ARD genou",
    "ARG hanche", "ARG cuisse", "ARG genou",
]


def on_lowstate(message):
    d = message["data"]

    # Efface l'écran pour un affichage qui se rafraîchit sur place
    sys.stdout.write("\033[H\033[J")

    print("=== Unitree Go2 - Capteurs ===\n")

    rpy = d["imu_state"]["rpy"]
    print(f"IMU  roll={rpy[0]:+.3f}  pitch={rpy[1]:+.3f}  yaw={rpy[2]:+.3f}  (rad)")

    print(f"\nForces aux pieds : {d['foot_force']}")

    bms = d["bms_state"]
    print(f"\nBatterie : {bms['soc']} %   courant={bms['current']} mA   tension={d['power_v']} V")

    print("\nMoteurs (angle en rad, température en °C) :")
    for i, m in enumerate(d["motor_state"][:12]):
        print(f"  {i:2} {NOMS_MOTEURS[i]:<11} q={m['q']:+.3f}   T={m['temperature']}")

    print("\nChamps disponibles :", list(d.keys()))
    sys.stdout.flush()


async def main():
    print(f"Connexion au robot {ROBOT_IP}...")
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP, aes_128_key=AES_KEY)
    await conn.connect()
    print("Connecté !")

    conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LOW_STATE"], on_lowstate)

    # Tourne jusqu'à Ctrl+C
    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nArrêt.")
