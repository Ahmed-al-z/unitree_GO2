"""
Caméra du Unitree Go2 en direct, SANS lag qui s'accumule (WebRTC / Wi-Fi). Lecture seule.

Différence avec l'exemple de la bibliothèque : on ne garde QUE la dernière image reçue.
Si l'affichage prend du retard, les vieilles images sont jetées au lieu de s'empiler
(dans l'exemple, une file d'attente sans limite faisait grandir le retard en continu).

Touches (dans la fenêtre vidéo) :
  q  quitter
  s  enregistrer une photo (dossier photos_go2/)

Lancer :  python go2_camera.py
"""
import asyncio
import logging
import os
import threading
import time

import cv2

from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod

ROBOT_IP = "192.168.2.6"
AES_KEY = "70920cdff469a9d6eee2a89a9bf15452"
LARGEUR_AFFICHAGE = 960          # réduire (ex. 640) si le PC est lent ; None = taille d'origine

logging.basicConfig(level=logging.FATAL)

derniere = {"img": None, "n": 0, "t": 0.0}      # seule la dernière image est conservée
verrou = threading.Lock()
etat = {"statut": "Connexion...", "fin": False}


async def recevoir(track):
    while not etat["fin"]:
        frame = await track.recv()
        img = frame.to_ndarray(format="bgr24")
        with verrou:
            derniere["img"] = img                    # écrase l'ancienne : pas d'accumulation
            derniere["n"] += 1
            derniere["t"] = time.time()


def thread_webrtc():
    async def main():
        conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP, aes_128_key=AES_KEY)
        await conn.connect()
        conn.video.switchVideoChannel(True)
        conn.video.add_track_callback(recevoir)
        etat["statut"] = "Connecté, attente de la vidéo..."
        while not etat["fin"]:
            await asyncio.sleep(0.5)
    try:
        asyncio.run(main())
    except Exception as e:
        etat["statut"] = f"Erreur : {e}"


def main():
    threading.Thread(target=thread_webrtc, daemon=True).start()
    os.makedirs("photos_go2", exist_ok=True)
    cv2.namedWindow("Go2 - camera", cv2.WINDOW_NORMAL)

    deja_affichee, compteur_fps, t_fps, fps = -1, 0, time.time(), 0.0
    while True:
        with verrou:
            img, n, t_img = derniere["img"], derniere["n"], derniere["t"]

        if img is None:
            print(f"\r{etat['statut']}   ", end="", flush=True)
            if cv2.waitKey(100) & 0xFF == ord("q"):
                break
            continue

        if n != deja_affichee:                       # nouvelle image -> on l'affiche
            deja_affichee = n
            compteur_fps += 1
            if time.time() - t_fps >= 1.0:
                fps = compteur_fps / (time.time() - t_fps)
                compteur_fps, t_fps = 0, time.time()
            vue = img
            if LARGEUR_AFFICHAGE and img.shape[1] > LARGEUR_AFFICHAGE:
                h = int(img.shape[0] * LARGEUR_AFFICHAGE / img.shape[1])
                vue = cv2.resize(img, (LARGEUR_AFFICHAGE, h), interpolation=cv2.INTER_AREA)
            else:
                vue = img.copy()
            texte = f"{img.shape[1]}x{img.shape[0]}  {fps:4.1f} img/s  (q: quitter, s: photo)"
            cv2.putText(vue, texte, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(vue, texte, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            cv2.imshow("Go2 - camera", vue)

        touche = cv2.waitKey(5) & 0xFF
        if touche == ord("q"):
            break
        if touche == ord("s"):
            chemin = os.path.join("photos_go2", time.strftime("photo_%Y%m%d_%H%M%S.jpg"))
            cv2.imwrite(chemin, img)
            print(f"\nPhoto enregistrée : {os.path.abspath(chemin)}")
        if cv2.getWindowProperty("Go2 - camera", cv2.WND_PROP_VISIBLE) < 1:
            break

    etat["fin"] = True
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    os._exit(0)
