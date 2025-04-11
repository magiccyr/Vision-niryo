from pyniryo import *
import tkinter as tk
import cv2
import numpy as np
import time

# Définition des positions de placement pour chaque couleur
colors = {
    "GREEN": PoseObject(0.0, 0.2, 0.2, 0.0, 1.57, 1.57),
    "RED": PoseObject(-0.15, 0.2, 0.2, 0.0, 1.57, 1.57),
    "BLUE": PoseObject(0.2, 0.2, 0.2, 0.0, 1.57, 1.57),
}
# Définition des plages de couleurs HSV pour la vision 
COLOR_HSV={
    "GREEN": ((35,50,50),(85,255,255)),
    "RED": ((0,50,50),(10,255,255)),
    "BLUE": ((100,50,50),(140,255,255)),
}
# Définition du nom de l’espace de travail
workspace_name = "default_workspace"
robot_ip_address = "192.168.0.105"

# Pose d'observation
observation_pose = PoseObject(
    x=0.25, y=0.0, z=0.30,
    roll=0.0, pitch=1.8, yaw=0.0,
)

# Connexion au robot
robot = NiryoRobot(robot_ip_address)
robot.calibrate_auto()
robot.tool_reboot()  # Redémarrage de l'outil (utile pour éviter des erreurs)
fenetre=tk.Tk()
fenetre.title("Interface graphique")
fenetre.geometry("400x300")
 
def object_detection():
    # Activating connexion with Conveyor Belt
    conveyor_id = robot.set_conveyor()
    robot.move_pose(observation_pose)

    # Running the Conveyor Belt at 50% of its maximum speed, in forward direction
    try:
        robot.run_conveyor(conveyor_id, speed=50, direction=ConveyorDirection.BACKWARD)
        start_time =time.time() # temps de départ

        while (time.time()-start_time<15):
            detected,_,_,_=robot.detect_object(workspace_name=workspace_name)

            if detected:
                robot.stop_conveyor(conveyor_id)
                tri_objet()
                start_time =time.time()
                
            #pause pour éviter de surcharger le processeur
            time.sleep(0.2) 
        # arrêt du convoyeur (que ce soit après 15s ou détection)    

    except KeyboardInterrupt:
        print("Arret manuel demandé")
    finally:
        # Stopping robot's motor    
        robot.stop_conveyor(conveyor_id)
        robot.unset_conveyor(conveyor_id)

def tri_objet():
    while True:
        found_something = False  # Variable pour vérifier si un objet a été trouvé

        for couleur, position in colors.items():
            robot.move_pose(observation_pose)
            robot.wait(1)

            # Tentative de prise avec la caméra
            obj_found, shape_ret, color_ret = robot.vision_pick(workspace_name, color=couleur)
            print(f"Détection : Forme={shape_ret}, Couleur={color_ret}")

            if obj_found:
                found_something = True
                robot.place_from_pose(position)
                robot.move_pose(observation_pose)
            
            # Si aucun objet n'a été trouvé dans toute la boucle, on sort de la boucle
        if not found_something:
            print("Aucun objet détecté, arrêt du programme.")
            break

def bouton_video():
    # Moving to observation pose
    robot.move_pose(observation_pose)

    while True:
        # Getting image
        img_compressed = robot.get_img_compressed()
        # Uncompressing image
        img_raw = uncompress_image(img_compressed)
        # - Display
        # Showing images
        cv2.imshow("video",img_raw)
        key = cv2.waitKey(30)& 0xFF
        if key in [27, ord("q")]:  # Will break loop if the user press Escape or Q
            break
    cv2.destroyAllWindows

def fonction_image(couleur):
# Moving to observation pose
    robot.move_pose(observation_pose)
    lower,upper=COLOR_HSV[couleur]

    while True:
        # Getting image
        img_compressed = robot.get_img_compressed()
        # Uncompressing image
        img_raw = uncompress_image(img_compressed)
        # conversion en HSV
        img_hsv=cv2.cvtColor(img_raw,cv2.COLOR_BGR2HSV)
        # FILTRAGE DE LA COULEUR
        mask=cv2.inRange(img_hsv,np.array(lower),np.array(upper))


        # Showing images
        cv2.imshow(f"Detection {couleur}",mask)
        key = cv2.waitKey(30)& 0xFF
        if key in [27, ord("q")]:  # Will break loop if the user press Escape or Q
            break

    cv2.destroyAllWindows

def bouton_vert():
    fonction_image("GREEN")
def bouton_rouge():
    fonction_image("RED")
def bouton_bleu():
    fonction_image("BLUE")
def bouton_quit():
    fenetre.quit()
    robot.close_connection()

bouton1=tk.Button(fenetre,text="Bouton video", command=bouton_video,font=("Arial",12),bg="gray")
bouton1.pack(pady=5)
bouton2=tk.Button(fenetre,text="Bouton video filtre vert", command=bouton_vert,font=("Arial",12),bg="green")
bouton2.pack(pady=5)
bouton3=tk.Button(fenetre,text="Bouton video filtre rouge", command=bouton_rouge,font=("Arial",12),bg="red")
bouton3.pack(pady=5)
bouton4=tk.Button(fenetre,text="Bouton video filtre bleu", command=bouton_bleu,font=("Arial",12),bg="blue")
bouton4.pack(pady=5)
bouton5=tk.Button(fenetre,text="tri d'objet", command=object_detection,font=("Arial",12),bg="lightblue")
bouton5.pack(pady=5)
bouton6=tk.Button(fenetre,text="Quitter", command=bouton_quit,font=("Arial",12),bg="red")
bouton6.pack(pady=5)

fenetre.mainloop()