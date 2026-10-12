"""
Chercher les traces d'IA dans le Unitree Go2 (WebRTC / Wi-Fi). Lecture seule.

1) Demande au robot quel "mode de mouvement" (contrôleur) est actif.
2) Écoute pendant DUREE secondes les canaux liés à l'IA et à la voix :
     - rt/gptflowfeedback : réponses de l'assistant vocal (le "GPT" du robot)
     - rt/servicestate    : services logiciels qui tournent
     - rt/audiohub/player/state, rt/multiplestate : état audio / réglages
   Pendant l'écoute, parle au robot (assistant vocal) pour voir si des messages arrivent.

Ne fait bouger AUCUN moteur.
Lancer :  python go2_ia.py
"""
import asyncio
import json
import os
import time

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

ROBOT_IP = "192.168.2.6"
AES_KEY = "70920cdff469a9d6eee2a89a9bf15452"
DUREE = 60

CANAUX = {
    "rt/gptflowfeedback":        "ASSISTANT IA (GPT)",
    "rt/servicestate":           "SERVICES",
    "rt/audiohub/player/state":  "AUDIO",
    "rt/multiplestate":          "RÉGLAGES",
}

deja_vu = {}   # pour n'afficher que les messages qui changent


def lisible(data):
    if isinstance(data, str):
        try:
            return json.dumps(json.loads(data), ensure_ascii=False)
        except ValueError:
            return data
    try:
        return json.dumps(data, ensure_ascii=False, default=str)
    except TypeError:
        return str(data)


def make_cb(topic):
    def cb(message):
        texte = lisible(message.get("data", message) if isinstance(message, dict) else message)
        if deja_vu.get(topic) == texte:
            return
        deja_vu[topic] = texte
        heure = time.strftime("%H:%M:%S")
        print(f"[{heure}] {CANAUX[topic]:<20} {texte[:300]}")
    return cb


async def main():
    print(f"Connexion au robot {ROBOT_IP}...")
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP, aes_128_key=AES_KEY)
    await conn.connect()
    print("Connecté !\n")

    # 1) Mode de mouvement actif (lecture uniquement)
    try:
        rep = await conn.datachannel.pub_sub.publish_request_new(
            "rt/api/motion_switcher/request", {"api_id": 1001})
        statut = rep["data"]["header"]["status"]["code"]
        if statut == 0:
            print("Mode de mouvement actif :", rep["data"]["data"])
        else:
            print(f"Mode de mouvement : requête refusée (code {statut})")
    except Exception as e:
        print(f"Mode de mouvement : pas de réponse ({e})")

    # 2) Écoute des canaux IA / voix
    for topic in CANAUX:
        conn.datachannel.pub_sub.subscribe(topic, make_cb(topic))

    print(f"\nÉcoute pendant {DUREE} s. Parle au robot maintenant (assistant vocal) !")
    print("Seuls les messages qui CHANGENT sont affichés.\n")
    await asyncio.sleep(DUREE)

    muets = [CANAUX[t] for t in CANAUX if t not in deja_vu]
    print("\nCanaux restés silencieux :", ", ".join(muets) if muets else "(aucun)")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nArrêt.")
    os._exit(0)
