# -*- coding: utf-8 -*-
"""
Ceci est un script de capture d'un flux video d'une camera zwo
Necessite l'installation de zwoasi de steve Marple' 
Utilise pySimpleGUI pour les fenetres definies dans le module myGUI
Extrait le centre de la raie en recherchant le minimum d'intensité pour 
chaque colonne
Travaille sur des images spectrales horizontales
Affiche en temps réel avec open CV et Thread pour affichage
Nom du fichier ser construit au format heure_min_sec pris au debut de l'acquisition
version 0.0.2 avec modif JB Butet serfilesreader module
version 0.0.3 avec modif JB Butet pynput
pysolex_rev2 avec correction bug retour variable exp gain dans le bon ordre depuis GUI et argmin dans la routine d'extraction du minimum

"""
import argparse
import os
import sys
import time
 #module de Steve Marpple pour lecture camera ZWO
import zwoasi as asi
import cv2
import numpy as np
#module de GUI
import PySimpleGUI as sg 
import datetime as dt
#import matplotlib.pyplot as plt
from astropy.io import fits
import threading as th
import queue

try : 
    from serfilesreader import Serfile
except: 
    from serfilesreader.serfilesreader import Serfile
    
from pynput import keyboard
#gestion de la date
from astropy.time import Time
from datetime import datetime



__author__ = 'Valerie desnoux'
__version__ = '0.0.2'


"""
-------------------------------------------------------------------------------------
"""
def on_press(key):
    global q_pressed
    try:
        #print('alphanumeric key{0}pressed'.format(key.char))
        pass
    except AttributeError:
        #print('special key{0}pressed'.format(key))
        pass

def on_release(key):
    global q_pressed
    if str(key).strip("'") == 'q':# Stop listener
        q_pressed=True
        print('q pressed')


# subroutine pour eventuellement sauvegarder les valeurs de controles de la camera
# n'est pas utilisé pour l'instant
def save_control_values(filename, settings):
    filename += '.txt'
    with open(filename, 'w') as f:
        for k in sorted(settings.keys()):
            f.write('%s: %s\n' % (k, str(settings[k])))
    print('Camera settings saved to %s' % filename)
    
def mise_en_forme (values):
    # mise forme des valeurs de ROI, Gain et Exp
    ROI=values ['-ROI-']
    Exp=values ['-EXP-']
    Gain=values ['-GAIN-']
    r=ROI.split(",")
    x1=int(r[0])
    y1=int(r[1])
    # attention les valeurs de ROI width et height doivent etre multiple de 8
    w=int(r[2])
    h=int(r[3])
    if w %8 !=0:
        w=w -w%8
    if h %8 !=0:
        h=h -h%8
    #on ne peut pas avoir une exposition en ms qui ne soit pas un entier
    if float(Exp)>=0.001:
        Exp=int(float(Exp)*1000)
    else:
        Exp=1
    Gain=int(Gain)
    return x1,y1,w,h,Gain, Exp
    
    
# subroutine pour capturer une seule image
def capture_one_image(FileName, Img_Gain, Img_Exp):
    #print(Img_Gain, Img_Exp)
    print('Appuyer sur touche clavier pour fermer image')
    camera.disable_dark_subtract()
    camera.set_control_value(asi.ASI_GAIN, Img_Gain)
    camera.set_control_value(asi.ASI_EXPOSURE, Img_Exp)
    camera.set_control_value(asi.ASI_WB_B, 99)
    camera.set_control_value(asi.ASI_WB_R, 75)
    camera.set_control_value(asi.ASI_GAMMA, 50)
    camera.set_control_value(asi.ASI_BRIGHTNESS, 50)
    camera.set_control_value(asi.ASI_FLIP, 0)
    filename = FileName
    camera.set_image_type(asi.ASI_IMG_RAW16)
    frame=camera.capture(filename=filename)
    print('Saved to %s' % filename)
    
    
    cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Image', cam_MaxWidth, cam_MaxHeight)
    cv2.moveWindow('Image', 10, 30)
    #on capture la trame image
    frame=np.array(frame, dtype='uint16')

    #on affiche Image en temps reel dans la fenetre
    cv2.imshow('Image', frame)
    cv2.waitKey(0)
    #on ferme la fenetre de video mais pas la camera
    cv2.destroyAllWindows()
    
"""
---------------------------------------------------------------------------
Thread d'affichage image spectre et contruction live du disk
q:      queue ou a été postée l'image acquise par le main
mode:   Preview ou Capture ne reconstruise pas le disque en direct
        Video reconstruit l'image avec l'extraction du minimum
ih:     Hauteur de l'image
iw:     Largeur de l'image
return r: queue de resultat 
Window spectre: fenetre d'affichage du spectre 
Window disk: fenetre d'affichage de l'image en construction
Utilise open CV
Arrete l'affichage et le process à la reception de 'None' dans la queue q
return 'None' en retour dans la queue r en mode Preview ou Capture
return le tableau Disk de l'image du disque recontruit dans la queue r
---------------------------------------------------------------------------
"""
    
def display_stream_spectre (q,mode,ih,iw,r):

    cv2.namedWindow('spectre', cv2.WINDOW_AUTOSIZE)
    Disk=np.zeros((iw,1), dtype='uint16')
    
    if mode=='Video':
        cv2.namedWindow('disk', cv2.WINDOW_NORMAL)
    ok_flag=True
    
    while(ok_flag):
        # recupere la trame que la caméra a posté, FIFO
        frame=q.get()
        
        if frame is None:
            
            # arret de l'acquisition donc on arrete
            ok_flag=False
            
            # important de fermer les fenetres openCV dans le Thread de création
            cv2.destroyAllWindows() 
            
            if mode=='Preview' or mode=='Capture':
                # n'a pas assemblé d'image du disque donc retourne None
                Disk=None
            r.put(Disk)

        else:
            # affiche le spectre
            cv2.imshow('spectre',frame) 
            cv2.waitKey(1)
                            
            # en mode video calcul les min des colonnes de chaque trame,
            # contruit le disque de l'image monochromatique
            # affichele disque qui se contruit à chaque trame
            
            if mode=='Video':

                IntensiteRaie=np.empty(iw,dtype='uint16')
                
                for j in range(0,iw):
                    col_h=frame[:,j]
                    MinX=col_h.argmin()
                    IntensiteRaie[j]=frame[MinX,j]
         
                #ajoute au tableau disk 
                IntensiteRaie=IntensiteRaie.reshape((iw,1))
                Disk=np.append(Disk,IntensiteRaie, axis=1)

                # affiche en temps réel le disque qui se construit
                cv2.resizeWindow('disk', Disk.shape[1], Disk.shape[0])
                cv2.imshow ('disk', Disk)
                cv2.waitKey(1)
       
        # on a traité l'element de la queue, elle peut proposer le suivant
        q.task_done()


"""
-----------------------------------------------------------------------------------------------------
initialisation camera
-----------------------------------------------------------------------------------------------------
"""
#variable environnement window definie ici  C:\ASI SDK\lib\x64\ASICamera2.dll
env_filename = os.getenv('ZWO_ASI_LIB') 

parser = argparse.ArgumentParser(description='Process and save images from a camera')
parser.add_argument('filename',
                    nargs='?',
                    help='SDK library filename')
args = parser.parse_args()

if env_filename is None :
    if os.name=='nt':
        env_filename = 'C:\ASI SDK\lib\x64\ASICamera2.dll'
    elif os.name=='posix':
        env_filename = "/usr/lib/x86_64-linux-gnu/libASICamera2.so.1.16.3"
# Initialize zwoasi with the name of the SDK library chez moi ici C:\ASI SDK\lib\x64\ASICamera2.dll
print(env_filename)
try:
    asi.init(env_filename)
except:
    #print("""Il faut installer le SDK ou configurer la DLL avec la variable d'environnement ZWO_ASI_LIB""")
    #sys.exit()
    pass
    
num_cameras = asi.get_num_cameras()
if num_cameras == 0:
    print('pas de caméra trouvée')
    time.sleep(2) 
    sys.exit(0)

cameras_found = asi.list_cameras()  # Models names of the connected cameras

if num_cameras == 1:
    camera_id = 0
    print('A trouvé une caméra: %s' % cameras_found[0])
else:
    print('A trouvé %d caméras' % num_cameras)
    for n in range(num_cameras):
        print('    %d: %s' % (n, cameras_found[n]))
    # TO DO: allow user to select a camera
    camera_id = 0
    print('avec ID #%d: %s' % (camera_id, cameras_found[camera_id]))

#declare la camera comme un objet de la classe asi
camera = asi.Camera(camera_id)

#recupere les proprietes comme la taille, le nom, la taille du pixel... 
#ne sert pas dans la suite du programe
camera_info = camera.get_camera_property()
#test d'affichage
print ('largeur :%s' % camera_info ['MaxWidth'], 'hauteur :%s' %camera_info ['MaxHeight'], 'Taille Pixel :%s' %camera_info ['PixelSize'])

#recupere les valeurs de controles comme le gain, l'exposition...
controls = camera.get_controls()
info_size=camera.get_roi ()
cam_MaxWidth=info_size[2]
cam_MaxHeight=info_size[3]

"""
--------------------------------------------------------------------------------------
on lance la fenetre pour recuperer les parametres d'acquisition et declencher actions
--------------------------------------------------------------------------------------
"""
ROI_full_init='0,0,'+str(camera_info ['MaxWidth'])+','+str(camera_info ['MaxHeight'])



"""
--------------------------------------------------------------------------------------
Gestion du clavier multiplateforme
--------------------------------------------------------------------------------------
"""
listener = keyboard.Listener(on_press=on_press,on_release=on_release)
listener.start()
q_pressed = False


# Aie ! oui ici je peux declarer en dur - TODO: fichier de config ini
#ROI_full_init='500,328,1000,88' 

sg.theme('Dark2')
sg.theme_button_color(('white', '#500000'))            

layout = [
[sg.Text('Nom du fichier', size=(16, 1)),
 sg.InputText(size=(25,1),default_text='c:\py\zwo',key='-FILE-'),
 sg.FileSaveAs(key='-SAVEAS-', file_types=(("SER Files", "*.ser"),),initial_folder='py', default_extension='.ser')],
[sg.Text('Fenetre x1,y1,largeur, Hauteur',size=(25,1)),
 sg.Input(size=(18,1),key='-ROI-',enable_events=True)],
[sg.Text('Exposition (ms)', size=(25, 1)), sg.InputText(default_text='15',size=(6,1),key='-EXP-'),sg.Button('Update')],
[sg.Text('Gain', size=(25,1)), sg.InputText(default_text='600',size=(6,1),key='-GAIN-')],
[sg.Button('Preview'),sg.Button('Video'),sg.Button('Capture'), sg.Button('Image'),sg.Button('Stop'),sg.Cancel()]
]

window = sg.Window('pyZwoSolex', layout, location=(800,100),keep_on_top=True,finalize=True)
window['-ROI-'].update(ROI_full_init) 
window.BringToFront()

# la Grande Boucle de l'UI
while True:
    event, values = window.read(timeout=10)
    
    if event==sg.WIN_CLOSED or event=='Cancel': 
        break
   
    # si on entre des caraceteres dans le champs ROI, test si caractere est ok
    if event == '-ROI-' and values['-ROI-'] and values['-ROI-'][-1] not in ('0123456789.,'):
        window['-ROI-'].update(values['-ROI-'][:-1])
    
    # si on click le bouton Image, capture une image
    if event=='Image':
        FileName=values['-FILE-']
        FileName= FileName.split(".")
        FileName=FileName[0]+".png"
        print (FileName)
        x1,y1,w,h,Gain,Exp=mise_en_forme(values)
        # defini la ROI de capture ... important de le faire avant le lancement du mode capture !
        camera.set_roi(x1,y1,w,h)
        info_size=camera.get_roi ()
        cam_MaxWidth=info_size[2]
        cam_MaxHeight=info_size[3]
        
        capture_one_image(FileName, Gain, Exp)
        
    # si on click l'un des boutons Video, Preview, Capture
    # on se met en position de capturer un flux video
    if event=='Video' or event=='Preview' or event=='Capture':
        if values['-FILE-'].lower().endswith('.ser')==False:
            values['-FILE-']=values['-FILE-']+'.ser'
           
        serfile=values['-FILE-']
        myWorkDir=os.path.dirname(serfile)
        
        x1,y1, w,h, Gain, Exp=mise_en_forme(values)
               
        # on forme le nom du fichier ser avec heure, min, sec
        now=dt.datetime.now()
        baseline=('%02d_%02d_%d'%(now.hour,now.minute,now.second))+'.ser'
        baseline=os.path.join(myWorkDir, baseline)
        window['-FILE-'].update(baseline) 

        """
        ---------------------------------------------------------------------------------------
        on prepare le mode video capture et on cree l'entete du fichier video .ser'
        ---------------------------------------------------------------------------------------
        """
        # Use met a USB 80%
        camera.set_control_value(asi.ASI_BANDWIDTHOVERLOAD, 80)
        # set Gain and Exposure
        camera.set_control_value(asi.ASI_EXPOSURE, Exp)
        camera.set_control_value(asi.ASI_GAIN, Gain)
    
        #on configure le mode en RAW16... tres important de le faire avant de se mettre en video capture !
        camera.set_image_type(asi.ASI_IMG_RAW16)
    
        # defini la ROI de capture ... important de le faire avant le lancement du mode capture !
        camera.set_roi(x1,y1,w,h)
        info_size=camera.get_roi ()
        cam_MaxWidth=info_size[2]
        cam_MaxHeight=info_size[3]
    
        #on se met en mode video capture
        camera.start_video_capture()
    
        # Set the timeout, units are ms - conformement a la recommandation dans sdk zwo
        timeout = (camera.get_control_value(asi.ASI_EXPOSURE)[0] / 1000) * 2 + 500
        camera.default_timeout = timeout
    
        # on confirme le mode d'acquisition video en 16 bits N&B et la ROI à nouveau sinon ROI reste au debut de l'image
        camera.set_image_type(asi.ASI_IMG_RAW16)
        camera.set_roi(x1,y1,w,h)
        info_size=camera.get_roi ()
        cam_MaxWidth=info_size[2]
        cam_MaxHeight=info_size[3]
        iw=cam_MaxWidth
        ih=cam_MaxHeight
        #print (info_size[0],info_size[1],cam_MaxWidth,cam_MaxHeight)

    
        #flag pour boucle acquisition video
        ok_flag=True
        
        #valeur de wait en seconde pour limiter les fps 
        max_fps=75
        limit_fps= 1/max_fps
        
        if event=='Capture' or event=='Video':
            serfile_object = Serfile(baseline, NEW=True)
            
            #camera type
            fileid=cameras_found[0]
            serfile_object.setFileID(fileid)
            
            #Largeur, hauteur, nb de bits, nb de frame
            Width=cam_MaxWidth
            serfile_object.setImageWidth(Width)
            Height=cam_MaxHeight
            serfile_object.setImageHeight(Height)
            serfile_object.setPixelDepthPerPlane(16)
        
            #observateur, Intrument, telescope
            s="valerie desnoux" #TODO a mettre dans la GUI
            serfile_object.setObserver(s)
            
            
            s='Exp :'+str(Exp/1000)+' Gain : '+str(Gain) 
            serfile_object.setInstrument(s)
            
            s="pySolexZwo" #TODO add optical information
            serfile_object.setTelescope(s)
        
            # date et date UTC
            date = Time.now().jd
            serfile_object.setDateTimeUTC(date)
            
            # date
            dateUTC = Time(datetime.utcnow(), scale='utc').jd
            serfile_object.setDateTimeUTC(dateUTC)
            
            

        
        """
        ----------------------------------------------------------------------
        ----------------------------------------------------------------------
        la on lance capture et le thread de display
        on est en mode Preview, Video ou Capture
        ----------------------------------------------------------------------
        """
        FrameCount=0
        # debug pour mesure temps
        #t_exec=[]  

        #initialize le tableau qui recevra l'image somme de toutes les trames
        mydata=np.zeros((ih,iw),dtype='uint64')
        
        #prepare les queues et lance le process de display
        q=queue.Queue(maxsize=0)    #queue pour transmettre frame au thread
        r=queue.Queue(maxsize=0)    #queue pour recevoir le disk reconstruit parle thread
        mode=event
        
        t=th.Thread(name='display_stream_spectre',target=display_stream_spectre, args=(q,mode,ih,iw,r,))
        t.start()
        print('process started')

        #pour calculer le frame rate
        TimeDebut=float(time.time()) 
        
        #on boucle tant qu'on n'a pas appuyé sur la touche 'q' du clavier ou condition ok_flag fausse
        while ok_flag:
            
            # on recupere le temps du systeme pour mesurer l'acquisition
            t0 = float(time.time())
            
            # mode preview, on affiche et on autorise la mise a jour gain, exp et ROI
            # on arrete le mode prview avec bouton Stop 
            
            if event=='Preview':
                # boucle pour lire les boutons update, stop pendant l'acquisition
                eventA,values=window.read(timeout=100)
                
                if eventA=='Update':
                    x1,y1,w,h,MyGain,MyExp = mise_en_forme(values)
                    camera.set_control_value(asi.ASI_EXPOSURE, MyExp)
                    camera.set_control_value(asi.ASI_GAIN, MyGain) 
                    camera.set_roi(x1,y1,w,h)
                    
                if eventA=='Stop':
                    ok_flag=False
  
                     
            #on capture la trame image
            frame=camera.capture_video_frame()
            frame=np.array(frame, dtype='uint16')
            # post la trame pour le thread de display
            q.put(frame)
            
            if ok_flag==False:
            # si on arrete l'acquisition
            # post None pour que le thread s'arrete
                q.put(None)
            
            # modes capture et video ecrivent le fichier ser
            if event=='Capture' or event=='Video': 
                mydata=np.add(frame,mydata)
                serfile_object.addFrame(mydata)
                
                # test si la touche 'q' a été appuyée pour arreter
                #if keyboard.is_pressed('q') or keyboard.is_pressed(' '):
                if q_pressed :
                    print('\a')       # beep !!             
                    ok_flag=False
                    q.put(None)
                    q_pressed=False
            
            FrameCount=FrameCount+1
            
            # on boucle pour attendre avant de capturer trame suivante
            # sinon on a des fps qui peuvent etre tres elevés
            end_time=t0+limit_fps
            t2=t0
            while t2<end_time:
                t2=float(time.time())
            
            #debug
            #t1=float(time.time())
            #t_exec.append(t1-t0)
            

          
        """
        ---------------------------------------------------------------------
        Sortie de l'acquisition
        ---------------------------------------------------------------------
        """
        # time stamp pour calcul fps
        TimeFin=float(time.time())
        # recupere le tableau Disk du thread - a None en mode Preview et Capture
        Disk=r.get()
        #q.join() #pas utile car affiche tant que q n'est pas à None
        
        # on stoppe le mode video capture
        camera.stop_video_capture()
        
        # debug
        #t_exec=t_exec[2:]
        #plt.plot(t_exec)
        #plt.show()
        #np.savetxt(baseline+'_exec.txt',t_exec)
        
        # decoupe le tableau Disk pour ne pas prendre premiere colonne
        if event=='Video':
            Disk=Disk[:,1:]
            #print(Disk.shape[0], Disk.shape[1])
        
        # calcul le frame rate
        print('fichier: ', baseline)
        print('Acquisition de %d frames:' % FrameCount)
        fps=FrameCount/(TimeFin-TimeDebut)
        print ("fps :", fps)

        
        if event=='Capture' or event=='Video':
            # Calcul de l'image moyenne
            myimg=mydata/(FrameCount-1)             # Moyenne
            myimg=np.array(myimg, dtype='uint16')   # Passe en entier 16 bits
            
            if ih<iw:
                print('rotate')
                myimg=np.rot90(myimg)
                
            if event=='Video':
                #affiche image assemblée dans fenetre ser
                cv2.namedWindow('Ser', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Ser', FrameCount, iw)
                cv2.moveWindow('Ser', 0, 0)
                cv2.imshow ('Ser', Disk)
                # attends appuie sur touche clavier
                print ('Appuyer sur touche clavier pour fermer fenetre openCV ser')
                print ('Fenetre ser peut-etre derriere les autres fenetres')
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                
                # creation entete fits
                hdr= fits.Header()
                hdr['SIMPLE']='T'
                hdr['BITPIX']=32
                hdr['NAXIS']=2
                hdr['NAXIS1']=iw
                hdr['NAXIS2']=FrameCount
                hdr['BZERO']=0
                hdr['BSCALE']=1
                hdr['BIN1']=1
                hdr['BIN2']=1
                hdr['EXPTIME']=Exp
                AxeY= hdr['NAXIS2']                    # Hauteur de l'image
                AxeX= hdr['NAXIS1']                    # Largeur de l'image
                Disk=np.reshape(Disk, (AxeX, AxeY))    # Forme tableau X,Y de l'image moyenne
                
                # Extrait nom du fichier sans extension
                base=os.path.basename(baseline)
                basefich=os.path.splitext(base)[0]
                savefich=os.path.join(myWorkDir,basefich+'_img')  
                
                # sauve en fits l'image reconstruite avec suffixe _img
                SaveHdu=fits.PrimaryHDU(Disk,header=hdr)
                SaveHdu.writeto(savefich+'.fits',overwrite=True)
                # sauve en png l'image reconstruite avec suffixe _img
                cv2.imwrite(savefich+'.png',Disk)
                
                # sauve en fits l'image moyenne avec suffixe _mean
                savefich=os.path.join(myWorkDir,basefich+'_mean')  
                hdr['NAXIS2']=iw
                SaveHdu=fits.PrimaryHDU(myimg,header=hdr)
                SaveHdu.writeto(savefich+'.fits',overwrite=True)
 
# on ferme la camera et la fenetre GUI 
camera.close()
window.close()

