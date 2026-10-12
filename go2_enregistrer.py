"""
Le robot te demande de parler, puis enregistre ta voix avec SON micro,
et sauvegarde l'enregistrement sur le PC (dossier enregistrements_go2/).

Déroulement :
  1. le robot dit (haut-parleur) : "Parle-moi maintenant, tu as X secondes."
  2. le micro du robot enregistre pendant DUREE_ENREGISTREMENT secondes
  3. le fichier .wav est sauvegardé sur le PC
  4. le robot dit : "Merci, c'est enregistré."
Appuie sur Entrée pour recommencer, "q" pour quitter.

Ne fait bouger AUCUN moteur.
Utilise go2_parler.py (doit être dans le même dossier) pour la voix du robot.
Lancer :  python go2_enregistrer.py
"""
import asyncio
import os
import time
import wave

import numpy as np

import go2_parler as parler                     # réutilise la voix du robot
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.webrtc_audiohub import WebRTCAudioHub

ROBOT_IP = parler.ROBOT_IP
AES_KEY = parler.AES_KEY

DUREE_ENREGISTREMENT = 5        # secondes
GAIN_MAX = 20                   # amplification maximale (x20 ≈ +26 dB)
COUPURE_HZ = 150                # filtre : on coupe les sons graves (ventilateurs) sous cette fréquence
DOSSIER = "enregistrements_go2"

micro = {"enregistre": False, "morceaux": [], "freq": 48000, "canaux": 2, "recu": 0}


async def recevoir_son(frame):
    """Appelée pour chaque petit bout de son venant du micro du robot."""
    micro["recu"] += 1
    if not micro["enregistre"]:
        return
    micro["freq"] = frame.sample_rate
    micro["canaux"] = len(frame.layout.channels)
    micro["morceaux"].append(frame.to_ndarray().astype(np.int16).tobytes())


def ameliorer(son, freq):
    """Filtre passe-haut (enlève le grondement des ventilateurs) + amplification."""
    # passe-haut du 1er ordre, coupure ~150 Hz (la voix est au-dessus)
    rc = 1.0 / (2 * np.pi * COUPURE_HZ)
    a = rc / (rc + 1.0 / freq)
    filtre = np.empty_like(son)
    filtre[0] = 0.0
    precedent_x, precedent_y = son[0], 0.0
    for i in range(1, len(son)):                  # boucle simple ; 5 s de son = rapide
        y = a * (precedent_y + son[i] - precedent_x)
        filtre[i], precedent_x, precedent_y = y, son[i], y
    # amplification : pic à ~70 % du maximum, limitée pour ne pas exploser le bruit
    pic = np.abs(filtre).max()
    gain = min(GAIN_MAX, 0.7 * 32767 / pic) if pic > 0 else 1.0
    return np.clip(filtre * gain, -32767, 32767), gain


def sauvegarder(chemin):
    donnees = b"".join(micro["morceaux"])
    brut = np.frombuffer(donnees, dtype=np.int16).astype(np.float64)
    if len(brut) == 0:
        return 0, 0, 1.0
    son = brut.reshape(-1, micro["canaux"]).mean(axis=1)   # le micro est mono : on fusionne les canaux
    niveau = float(np.sqrt(np.mean(son ** 2)))
    son, gain = ameliorer(son, micro["freq"])
    with wave.open(chemin, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)                       # 16 bits
        w.setframerate(micro["freq"])
        w.writeframes(son.astype(np.int16).tobytes())
    return len(son) / micro["freq"], niveau, gain


async def une_prise(hub, numero):
    await parler.dire(hub, f"Parle-moi maintenant. Tu as {DUREE_ENREGISTREMENT} secondes.")

    micro["morceaux"].clear()
    micro["enregistre"] = True
    for r in range(DUREE_ENREGISTREMENT, 0, -1):
        print(f"\r   >>> ENREGISTREMENT ... {r} s   ", end="", flush=True)
        await asyncio.sleep(1)
    micro["enregistre"] = False
    print()

    os.makedirs(DOSSIER, exist_ok=True)
    chemin = os.path.join(DOSSIER, time.strftime(f"voix_%Y%m%d_%H%M%S.wav"))
    duree, niveau, gain = sauvegarder(chemin)
    if duree == 0:
        print("   Aucun son reçu du micro du robot.")
        return
    print(f"   Enregistré : {os.path.abspath(chemin)}  ({duree:.1f} s, amplifié x{gain:.1f})")
    if gain >= GAIN_MAX:
        print("   (son d'origine très faible : parle plus près de la tête du robot)")

    await parler.dire(hub, "Merci, c'est enregistré.")


async def main():
    print(f"Connexion au robot {ROBOT_IP}...")
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP, aes_128_key=AES_KEY)
    await conn.connect()
    hub = WebRTCAudioHub(conn)

    conn.audio.switchAudioChannel(True)         # active le micro du robot
    conn.audio.add_track_callback(recevoir_son)
    print("Connecté ! Vérification du micro...")
    await asyncio.sleep(2)
    if micro["recu"] == 0:
        print("ATTENTION : aucun son ne vient encore du micro du robot. On essaie quand même.\n")
    else:
        print("Micro du robot OK.\n")

    numero = 1
    while True:
        choix = (await asyncio.to_thread(input, "Entrée = le robot t'écoute, q = quitter : ")).strip().lower()
        if choix == "q":
            break
        try:
            await une_prise(hub, numero)
            numero += 1
        except Exception as e:
            print(f"   Erreur : {e}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nArrêt.")
    os._exit(0)
