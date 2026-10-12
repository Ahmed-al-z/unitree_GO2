"""
Faire parler le Unitree Go2 par son haut-parleur (WebRTC / Wi-Fi).

Tu tapes une phrase dans le terminal -> le PC la transforme en voix (synthèse vocale
Windows, voix française) -> le fichier son est envoyé au "hub audio" du robot
-> le robot la joue sur son haut-parleur -> le fichier est ensuite effacé du robot.

Ne fait bouger AUCUN moteur.
Prérequis (une seule fois) :  pip install pyttsx3
Lancer :                      python go2_parler.py
Taper "q" pour quitter.
"""
import asyncio
import contextlib
import io
import json
import logging
import os
import time
import warnings
import wave

import pyttsx3
warnings.filterwarnings("ignore", message="Couldn't find ffmpeg")   # inutile ici : on ne traite que du WAV
from pydub import AudioSegment

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.webrtc_audiohub import WebRTCAudioHub

ROBOT_IP = "192.168.2.6"
AES_KEY = "70920cdff469a9d6eee2a89a9bf15452"

VOIX = "fr-FR"              # langue de la voix (fr-FR = Hortense, en-US = Zira)
VITESSE_VOIX = 170          # mots par minute (défaut Windows ~200)
FREQ = 44100                # Hz, format attendu par le hub audio
DOSSIER = "sons_go2"

logging.basicConfig(level=logging.ERROR)


# ---------------------------------------------------------------- synthèse vocale (sur le PC)

def texte_vers_wav(texte, chemin):
    moteur = pyttsx3.init()
    for v in moteur.getProperty("voices"):
        if any(VOIX.lower() in str(l).lower() for l in v.languages) or VOIX.lower() in v.id.lower().replace("_", "-"):
            moteur.setProperty("voice", v.id)
            break
    moteur.setProperty("rate", VITESSE_VOIX)
    brut = chemin + ".brut.wav"
    moteur.save_to_file(texte, brut)
    moteur.runAndWait()
    moteur.stop()
    son = AudioSegment.from_wav(brut).set_frame_rate(FREQ).set_channels(1).set_sample_width(2)
    son.export(chemin, format="wav")
    os.remove(brut)
    with wave.open(chemin) as w:
        return w.getnframes() / w.getframerate()        # durée en secondes


# ---------------------------------------------------------------- hub audio (sur le robot)

async def liste_sons(hub):
    rep = await hub.get_audio_list()
    try:
        return json.loads(rep["data"]["data"]).get("audio_list", [])
    except (KeyError, TypeError, ValueError):
        return []


async def dire(hub, texte):
    os.makedirs(DOSSIER, exist_ok=True)
    nom = f"phrase_{int(time.time())}"
    chemin = os.path.join(DOSSIER, nom + ".wav")

    print("   1/3 synthèse vocale sur le PC...")
    duree = await asyncio.to_thread(texte_vers_wav, texte, chemin)

    print(f"   2/3 envoi au robot ({duree:.1f} s de son, quelques secondes)...")
    with contextlib.redirect_stdout(io.StringIO()):      # la bibliothèque affiche chaque morceau envoyé
        await hub.upload_audio_file(chemin)

    son = None
    for _ in range(10):                                  # le robot met parfois un instant à l'enregistrer
        son = next((s for s in await liste_sons(hub) if s.get("CUSTOM_NAME") == nom), None)
        if son:
            break
        await asyncio.sleep(0.5)
    if not son:
        print("   le robot n'a pas confirmé la réception du son.")
        return

    print("   3/3 le robot parle...")
    await hub.play_by_uuid(son["UNIQUE_ID"])
    await asyncio.sleep(duree + 1.0)
    await hub.delete_record(son["UNIQUE_ID"])            # ne pas encombrer la mémoire du robot
    os.remove(chemin)


async def main():
    print(f"Connexion au robot {ROBOT_IP}...")
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP, aes_128_key=AES_KEY)
    await conn.connect()
    hub = WebRTCAudioHub(conn)
    print("Connecté !\n")
    print("Tape une phrase et appuie sur Entrée : le robot la dira. (q pour quitter)\n")

    while True:
        texte = (await asyncio.to_thread(input, "Toi -> robot : ")).strip()
        if texte.lower() == "q":
            break
        if not texte:
            continue
        try:
            await dire(hub, texte)
        except Exception as e:
            print(f"   Erreur : {e}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nArrêt.")
    os._exit(0)
