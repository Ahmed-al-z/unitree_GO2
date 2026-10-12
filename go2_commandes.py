"""
Test de l'API de commande du Unitree Go2 (WebRTC / Wi-Fi).

Menu interactif avec uniquement des commandes SÛRES : se lever, se coucher,
s'asseoir, gestes, petits déplacements lents. Pas de saut, pas de salto.

AVANT DE LANCER :
  - robot au sol, au moins 2 m dégagés tout autour, personne à côté ;
  - garde la télécommande en main pour reprendre la main à tout moment ;
  - ferme l'appli Unitree Go sur le téléphone.

Lancer :  python go2_commandes.py
"""
import asyncio
import json
import os
import random
import time

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.constants import RTC_TOPIC

ROBOT_IP = "192.168.2.6"
AES_KEY = "70920cdff469a9d6eee2a89a9bf15452"

# Vitesses volontairement faibles
VITESSE = 0.3        # m/s
VITESSE_ROT = 0.5    # rad/s
DUREE_MOUVEMENT = 1.5  # s

# Identifiants des commandes (identiques en mode "normal" et en mode "mcf")
CMD = {
    "StopMove": 1003, "StandUp": 1004, "StandDown": 1005, "BalanceStand": 1002,
    "RecoveryStand": 1006, "Euler": 1007, "Move": 1008, "Sit": 1009,
    "RiseSit": 1010, "Hello": 1016, "Stretch": 1017, "Heart": 1036, "GetState": 1034,
}

MENU = [
    ("1", "Se lever (StandUp + BalanceStand)"),
    ("2", "Se coucher (StandDown)"),
    ("3", "S'asseoir (Sit)"),
    ("4", "Se relever d'assis (RiseSit)"),
    ("5", "Dire bonjour (Hello)"),
    ("6", "S'étirer (Stretch)"),
    ("7", "Faire un cœur (Heart)"),
    ("8", f"Avancer   ({VITESSE} m/s pendant {DUREE_MOUVEMENT} s)"),
    ("9", f"Reculer   ({VITESSE} m/s pendant {DUREE_MOUVEMENT} s)"),
    ("10", f"Pas de côté à gauche ({VITESSE} m/s)"),
    ("11", f"Pas de côté à droite ({VITESSE} m/s)"),
    ("12", f"Tourner à gauche ({VITESSE_ROT} rad/s)"),
    ("13", f"Tourner à droite ({VITESSE_ROT} rad/s)"),
    ("14", "Incliner le corps (Euler : regarder en bas puis revenir)"),
    ("15", "Lire l'état du robot (GetState)"),
    ("0", "STOP : arrêter tout mouvement"),
    ("q", "Quitter"),
]


async def appeler(conn, api_id, parametre=None):
    """Commande avec réponse. Renvoie (code, données). code 0 = OK."""
    payload = {"api_id": api_id}
    if parametre is not None:
        payload["parameter"] = parametre
    reponse = await conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["SPORT_MOD"], payload)
    entete = reponse.get("data", {}).get("header", {})
    code = entete.get("status", {}).get("code", -1)
    return code, reponse.get("data", {}).get("data", "")


def envoyer_sans_reponse(conn, api_id, parametre):
    """Commande 'tir et oublie', utilisée pour Move (envoyée en continu)."""
    ident = int(time.time() * 1000) % 2147483648 + random.randint(0, 1000)
    conn.datachannel.pub_sub.publish_without_callback(RTC_TOPIC["SPORT_MOD"], {
        "header": {"identity": {"id": ident, "api_id": api_id},
                   "policy": {"priority": 0, "noreply": True}},
        "parameter": json.dumps(parametre),
        "binary": [],
    })


async def bouger(conn, vx, vy, vyaw, duree=DUREE_MOUVEMENT):
    """Envoie la consigne de vitesse 10 fois par seconde pendant 'duree', puis stoppe."""
    fin = time.time() + duree
    while time.time() < fin:
        envoyer_sans_reponse(conn, CMD["Move"], {"x": vx, "y": vy, "z": vyaw})
        await asyncio.sleep(0.1)
    return await appeler(conn, CMD["StopMove"])


async def executer(conn, choix):
    if choix == "1":
        code, _ = await appeler(conn, CMD["StandUp"])
        await asyncio.sleep(2)
        return await appeler(conn, CMD["BalanceStand"])
    if choix == "2":
        return await appeler(conn, CMD["StandDown"])
    if choix == "3":
        return await appeler(conn, CMD["Sit"])
    if choix == "4":
        return await appeler(conn, CMD["RiseSit"])
    if choix == "5":
        return await appeler(conn, CMD["Hello"])
    if choix == "6":
        return await appeler(conn, CMD["Stretch"])
    if choix == "7":
        return await appeler(conn, CMD["Heart"])
    if choix == "8":
        return await bouger(conn, VITESSE, 0, 0)
    if choix == "9":
        return await bouger(conn, -VITESSE, 0, 0)
    if choix == "10":
        return await bouger(conn, 0, VITESSE, 0)
    if choix == "11":
        return await bouger(conn, 0, -VITESSE, 0)
    if choix == "12":
        return await bouger(conn, 0, 0, VITESSE_ROT, 2.0)
    if choix == "13":
        return await bouger(conn, 0, 0, -VITESSE_ROT, 2.0)
    if choix == "14":
        await appeler(conn, CMD["Euler"], {"x": 0, "y": 0.2, "z": 0})
        await asyncio.sleep(2)
        return await appeler(conn, CMD["Euler"], {"x": 0, "y": 0, "z": 0})
    if choix == "15":
        cles = ["state", "bodyHeight", "speedLevel", "gait", "continuousGait", "economicGait"]
        code, donnees = await appeler(conn, CMD["GetState"], cles)
        if code == 0 and donnees:
            try:
                for k, v in json.loads(donnees).items():
                    print(f"     {k:<15} : {v}")
            except (ValueError, AttributeError):
                print(f"     {donnees}")
        return code, donnees
    if choix == "0":
        return await appeler(conn, CMD["StopMove"])
    print("  Choix inconnu.")
    return None


async def main():
    print("=" * 60)
    print(" ATTENTION : le robot va bouger.")
    print(" - au moins 2 m dégagés autour, personne à côté")
    print(" - garde la télécommande en main")
    print("=" * 60)
    await asyncio.to_thread(input, "Appuie sur Entrée quand c'est prêt...")

    print(f"Connexion au robot {ROBOT_IP}...")
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP, aes_128_key=AES_KEY)
    await conn.connect()
    print("Connecté !")

    # Mode de mouvement actuel (lecture seule, on ne le change pas)
    try:
        rep = await conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1001})
        if rep["data"]["header"]["status"]["code"] == 0:
            print(f"Mode de mouvement actuel : {json.loads(rep['data']['data']).get('name')}")
    except Exception as e:
        print(f"(Mode de mouvement inconnu : {e})")

    while True:
        print("\nCommandes :")
        for touche, texte in MENU:
            print(f"  {touche:>3}  {texte}")
        choix = (await asyncio.to_thread(input, "\nTon choix : ")).strip().lower()
        if choix == "q":
            print("Arrêt du robot avant de quitter...")
            try:
                await appeler(conn, CMD["StopMove"])
            except Exception:
                pass
            break
        try:
            resultat = await executer(conn, choix)
        except Exception as e:
            print(f"  Erreur : {e}")
            continue
        if resultat is not None:
            code = resultat[0]
            if code == 0:
                print("  -> OK")
            else:
                print(f"  -> refusé (code {code}). Le robot est-il debout ? Essaie d'abord la commande 1.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nArrêt.")
    os._exit(0)
