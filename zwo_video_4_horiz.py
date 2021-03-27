# -*- coding: utf-8 -*-
"""
Ceci est un script de capture d'un flux video d'une camera zwo
Necessite l'installation de zwoasi de steve Marple' 
Utilise pySimpleGUI pour les fenetres definies dans le module myGUI
Extrait le centre de la raie en recherchant le minimum d'intensité pour 
chaque colonne
Travaille sur des images spectrales horizontales
Affiche en temps réel avec open CV
Nom du fichier ser construit au format heure_min_sec pris au debut de l'acquisition

"""
import argparse
import os
import sys
import time
import zwoasi as asi
import cv2
import numpy as np
import PySimpleGUI as sg
import datetime as dt
import matplotlib.pyplot as plt
from astropy.io import fits
#import Solex_recon as sol


__author__ = 'Valerie desnoux'
__version__ = '0.0.0'


# subroutine pour eventuellement sauvegarder les valeurs de controles de la camera
# n'est pas utilisé pour l'instant
def save_control_values(filename, settings):
    filename += '.txt'
    with open(filename, 'w') as f:
        for k in sorted(settings.keys()):
            f.write('%s: %s\n' % (k, str(settings[k])))
    print('Camera settings saved to %s' % filename)
    
# subroutine pour capturer une seule image
def capture_one_image(FileName, Img_Gain, Img_Exp):
    print(Img_Gain, Img_Exp)
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

print(env_filename)
# Initialize zwoasi with the name of the SDK library chez moi ici C:\ASI SDK\lib\x64\ASICamera2.dll
try:
    asi.init(env_filename)
except:
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
#ROI_full_init='0,0,'+str(camera_info ['MaxWidth'])+','+str(camera_info ['MaxHeight'])
ROI_full_init='500,328,1000,88'
#FrameMax=3000

# todo gerer le repertoire et les noms de fichiers png ou tiff 

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
[sg.Text('FrameMax', size=(25,1)), sg.InputText(default_text='11000',size=(6,1),key='-FRMAX-')],
[sg.Button('Preview'),sg.Button('Video'),sg.Button('Capture'), sg.Button('Image'),sg.Button('Stop'),sg.Cancel()]
]

window = sg.Window('pyZwoSolex', layout, location=(800,100),keep_on_top=True,finalize=True)
window['-ROI-'].update(ROI_full_init) 
window.BringToFront()
    
while True:
    event, values = window.read(timeout=10)
    
    if event==sg.WIN_CLOSED or event=='Cancel': 
        break
   
    if event == '-ROI-' and values['-ROI-'] and values['-ROI-'][-1] not in ('0123456789.,'):
        window['-ROI-'].update(values['-ROI-'][:-1])
    
    
    if event=='Image':
        FileName=values['-FILE-']
        FileName= FileName.split(".")
        FileName=FileName[0]+".png"
        print (FileName)
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
        capture_one_image(FileName, Gain, Exp)
    
    if event=='Video' or event=='Preview' or event=='Capture':
        if values['-FILE-'].lower().endswith('.ser')==False:
            values['-FILE-']=values['-FILE-']+'.ser'
           
        serfile=values['-FILE-']
        myWorkDir=os.path.dirname(serfile)
        ROI=values ['-ROI-']
        Exp=values ['-EXP-']
        Gain=values ['-GAIN-']
        FrameMax=int(values['-FRMAX-'])
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
        now=dt.datetime.now()
        baseline=('%02d_%02d_%d'%(now.hour,now.minute,now.second))+'.ser'
        baseline=os.path.join(myWorkDir, baseline)
        window['-FILE-'].update(baseline) 

        """
        ---------------------------------------------------------------------------------------
        on prepare le mode video capture et on cree l'entete du fichier video .ser'
        ---------------------------------------------------------------------------------------
        """
        # Use met a USB 60%
        camera.set_control_value(asi.ASI_BANDWIDTHOVERLOAD, 60)
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
    
        #on cree et redimensionne une fenetre pour afficher les images
        cv2.namedWindow('video', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('video', cam_MaxWidth, cam_MaxHeight)
        cv2.moveWindow('video', 0, 0)
    
        #nom generique des images tiff sauvées, flag et nom fichier ser final
        filegen="image"
        ok_flag=True
        
        
        if event=='Capture' or event=='Video':
            #now=dt.datetime.now()
            #baseline=('%02d_%02d_%d'%(now.hour,now.minute,now.second))+'.ser'
            #baseline=os.path.join(myWorkDir, baseline)
            #window['-FILE-'].update(baseline) 
            
            #on ecrit l'entete du fichier ser
            f=open(baseline,"wb")
        
            #camera type
            s="ZWO-ASI-290min"
            b=bytearray(s.encode())
            b=np.array(b,dtype='int8')
            f.write(b)
        
            #LUID, ColorID,Little_endian
            num=np.array([0,0,0],dtype=np.uint32)
            f.write(num)
        
            #Largeur, hauteur, nb de bits, nb de frame
            Width=cam_MaxWidth
            Height=cam_MaxHeight
            PixelDepthPerPlane=16
            FrameCount=1
            num=np.array([Width,Height,PixelDepthPerPlane,FrameCount],dtype=np.uint32)
            f.write(num)
        
            #observateur, Intrument, telescope
            s="valerie desnoux"
            b=bytearray(s.encode())
            for i in range(len(b),40):
                b.append(0)
            b=np.array(b,dtype='int8')
            f.write(b)
            s='Exp :'+str(Exp/1000)+' Gain : '+str(Gain)
            b=bytearray(s.encode())
            for i in range(len(b),40):
                b.append(0)
            b=np.array(b,dtype='int8')
            f.write(b)
            s="pySolexZwo"
            b=bytearray(s.encode())
            for i in range(len(b),40):
                b.append(0)
            b=np.array(b,dtype='int8')
            f.write(b)
        
            # date et date UTC ici a zero
            num=np.array([0,0],dtype=np.uint64)
            f.write(num)
        
        ok_resize=True
        
        if event=='Video':
            cv2.namedWindow('disk', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('disk', FrameMax, iw)
            cv2.moveWindow('disk', 0, 100)
            #initialize le tableau qui va recevoir la raie spectrale de chaque trame
            Disk=np.zeros((iw,FrameMax), dtype='uint16')

        """
        ----------------------------------------------------------------------
        la on passe en mode preview ou mode video ou mode capture sequence
        ----------------------------------------------------------------------
        """
        FrameCount=0
        TimeDebut=float(time.time())
        t_exec=[]
        #tps=(Exp/1000000)+0.03       

        #initialize le tableau qui recevra l'image somme de toutes les trames
        mydata=np.zeros((ih,iw),dtype='uint64')
    
        #on boucle tant qu'on n'a pas appuyé sur la touche exit, space bar ou condition ok_flag fausse
        while ok_flag:
            # on recupere le temps du systeme pour cadencer l'acquisition
            t0 = float(time.time())
            
            # mode preview, on affiche et on autorise la mise a jour gain, exp et ROI
            if event=='Preview':
                eventA,values=window.read(timeout=100)
                if eventA=='Update':
                    MyExp=values ['-EXP-']
                    MyGain=values ['-GAIN-']
                    MyROI=values ['-ROI-']
                    # manage ROI format
                    r=MyROI.split(",")
                    x1=int(r[0])
                    y1=int(r[1])
                    # les valeurs de ROI width et height doivent etre multiple de 8
                    w=int(r[2])
                    h=int(r[3])
                    if w %8 !=0:
                        w=w -w%8
                    if h %8 !=0:
                        h=h -h%8
                    # manage exp format
                    if float(MyExp)>=0.001:
                        MyExp=int(float(MyExp)*1000)
                    else:
                        MyExp=1
                    # manage gain format
                    MyGain=int(MyGain)
                    print(MyExp, MyGain)
                    camera.set_control_value(asi.ASI_EXPOSURE, MyExp)
                    camera.set_control_value(asi.ASI_GAIN, MyGain) 
                    camera.set_roi(x1,y1,w,h)
                if eventA=='Stop':
                    ok_flag=False
            
            #on capture la trame image
            frame=camera.capture_video_frame()
            frame=np.array(frame, dtype='uint16')
            
            #mode capture sans visu pour aller leplus vite possible
            if event=='Capture': 
                mydata=np.add(frame,mydata)
                f.write(frame)
                
            #mode capture avec visu temps reel du disque qui se construit
            if event=='Video':
                mydata=np.add(frame,mydata)
                #f.write(frame)
                IntensiteRaie=np.empty(iw,dtype='uint16')
                
                for j in range(0,iw):
                    col_h=frame[:,j]
                    MinX=col_h.argmax()
                    IntensiteRaie[j]=frame[MinX,j]
         
                #ajoute au tableau disk 
                Disk[:,FrameCount]=IntensiteRaie

                # affiche en temps réel le disque qui se construit
                cv2.imshow ('disk', Disk)
                # exit if Escape is hit
                if cv2.waitKey(1) == 27 or cv2.waitKey(1) == 32:                    
                    ok_flag=False
            
            FrameCount=FrameCount+1
            if FrameCount>=FrameMax and event=='Video':
                ok_flag=False
            
            #on affiche spectre en temps reel dans la fenetre
            cv2.imshow('video', frame)
            # exit if Escape or space bar are hit
            if cv2.waitKey(1) == 27 or cv2.waitKey(1) == 32:    
                ok_flag=False
                
                
            t1=float(time.time())
            t_exec.append(t1-t0)
          
        # sortie d'acquisition
        cv2.destroyAllWindows()
        
        plt.plot(t_exec)
        plt.show()
        np.savetxt(baseline+'_exec.txt',t_exec)
        
        # decoupe le tableau Disk au bon nb de colonne= nb de frame
        if event=='Video':
            Disk=Disk[:,:FrameCount]
        
        # calcul le frame rate
        TimeFin=float(time.time())
        print('fichier: ', baseline)
        print('Acquisition de %d frames:' % FrameCount)
        fps=FrameCount/(TimeFin-TimeDebut)
        print ("fps :", fps)

        #on stoppe le mode video capture
        camera.stop_video_capture()
        
        if event=='Capture' or event=='Video':
            #on met a jour le FrameCount  et on ferme le fichier ser
            f.seek (38)
            FrameNb=np.array([FrameCount], dtype='uint32')
            f.write(FrameNb)
            f.close()

            # Ferme les fenetres cv2 
            cv2.destroyAllWindows()
        
            
            # Calcul de l'image moyenne
            myimg=mydata/(FrameCount-1)             # Moyenne
            myimg=np.array(myimg, dtype='uint16')   # Passe en entier 16 bits
            
            if ih<iw:
                print('rotate')
                myimg=np.rot90(myimg)
                #s=ih
                #ih=iw
                #iw=s
                
            if event=='Video':
                #affiche image assemblée
                cv2.namedWindow('Ser', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Ser', FrameCount, iw)
                cv2.moveWindow('Ser', 10, 0)
                cv2.imshow ('Ser', Disk)
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
                AxeX= hdr['NAXIS1']                     # Largeur de l'image
                Disk=np.reshape(Disk, (AxeX, AxeY))   # Forme tableau X,Y de l'image moyenne
                
                # Extrait nom du fichier sans extension
                # Nom du fichier de l'image moyenne
                base=os.path.basename(baseline)
                basefich=os.path.splitext(base)[0]
                savefich=os.path.join(myWorkDir,basefich+'_ima')  
                
                # sauve en fits l'image assemblée avec suffixe _im
                SaveHdu=fits.PrimaryHDU(Disk,header=hdr)
                SaveHdu.writeto(savefich+'.fit',overwrite=True)
                cv2.imwrite(savefich+'.png',Disk)
                
    
    
                """
                savefich=os.path.join(myWorkDir,basefich+'_mean')  
                hdr['NAXIS2']=iw
                
                # sauve en fits l'image moyenne avec suffixe _mean
                SaveHdu=fits.PrimaryHDU(myimg,header=hdr)
                SaveHdu.writeto(savefich+'.fit',overwrite=True)
                
                #affiche image moyenne
                cv2.namedWindow('Ser', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Ser', AxeX, AxeY)
                cv2.moveWindow('Ser', 10, 0)
                cv2.imshow ('Ser', myimg)
                if cv2.waitKey(0)==32:
                    # appel au module d'extraction, reconstruction et correction
                    cv2.destroyAllWindows()
                    Shift=0
                    FileGen='s'
                    sol.solex_proc(FileGen, 1, FrameCount-1,baseline,Shift)
                else:
                    cv2.destroyAllWindows()
                """
            
            
camera.close()
window.close()
