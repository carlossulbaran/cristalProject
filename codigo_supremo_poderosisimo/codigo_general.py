#importar la libreria
import pygame as pg
import serial,time
import numpy as np
import math as mt
import RPi.GPIO as gpio

from adafruit_servokit import ServoKit


#Inicializar pca
kit = ServoKit(channels=16)
#kit.servo[15].actuation_range = 180
#kit.servo[15].set_pulse_width_range(125, 512)

#Inicializar pines para sacar y meter el brazo del sensor
gpio.setup(4, gpio.OUT)
gpio.setup(17, gpio.OUT)

#Inicializar los pines para poder controlar los modos del arduino
gpio.setup(26, gpio.OUT)
gpio.setup(19, gpio.OUT)



#Funciones

#Inicializar el sistema
def inicializar():
    servo(120)
    time.sleep(3)
    desactivar()
    time.sleep(3)

    ubicacion = np.array([0,0])

    ancho, largo = HMI()

    posicion_muestras = muestras(10,ancho,largo)

    

    print("inicializacion completa")

    return ancho,largo,posicion_muestras,ubicacion

#Funcion para el HMI inicialretorna el ancho y largo del mapa
def HMI():
    # initialize the pygame module
    pg.init()
    # create a surface on screen that has the size of 240 x 180
    screen = pg.display.set_mode((500,500))

    #info 1
    largo = ""

    #chosee the back ground color
    screen.fill((135, 206, 235))

    class Button:
        def __init__(self, text,color_letra,color_boton):
            font = pg.font.SysFont("Arial",60)
            self.text = font.render(text, 1, pg.Color(color_letra))
            self.size = self.text.get_size()
            self.surface = pg.Surface(self.size)
            self.surface.fill(color_boton)
            self.surface.blit(self.text,(0,0))

    class Texto:
        def __init__(self, text,color_letra):
            font = pg.font.SysFont("Arial",70)
            self.text = font.render(text, 1, pg.Color(color_letra))
            self.size = self.text.get_size()

    # load and set the logo
    logo = pg.image.load("logo.png")
    pg.display.set_icon(logo)
    pg.display.set_caption("Proyecto cristal")
   
    #Pagina principal
    start = Button("Start","Black",(255,255,255))
    screen.blit(start.surface, (200,400))

    Nombre_proyecto = Texto("Proyecto cristal","Black")
    screen.blit(Nombre_proyecto.text, (50,50))

    image = pg.image.load("logo.png")

    image = pg.transform.scale(image,(200,200))

    screen.blit(image, (150,180))

    #update the screen
    pg.display.flip()


    running = True
     
    # main loop
    while running:
        # event handling, gets all event from the event queue
        for event in pg.event.get():
            # only do something if the event is of type QUIT
            if event.type == pg.QUIT:
                # change the value to False, to exit the main loop
                running = False
            if event.type == pg.MOUSEBUTTONDOWN:
                if pg.mouse.get_pressed()[0]:
                    screen.fill((135, 206, 235))
                    running = False
        pg.display.flip()

    #Pedir datos 1
    info1 = Texto("Cual es el largo?","Black")
    screen.blit(info1.text, (50,50))


    #update the screen
    pg.display.flip()

    running = True
     
    # main loop
    while running:
        # event handling, gets all event from the event queue
        for event in pg.event.get():
            # only do something if the event is of type QUIT
            if event.type == pg.QUIT:
                # change the value to False, to exit the main loop
                running = False
        
            if event.type == pg.KEYDOWN:
                largo += event.unicode
        
            if event.type == pg.MOUSEBUTTONDOWN:
                if pg.mouse.get_pressed()[0]:
                    screen.fill((135, 206, 235))
                    running = False
        

            info1_surface = Texto(largo,"Black")
            screen.blit(info1_surface.text, (50,150))

            #update the screen
            pg.display.flip()

    largo = int(largo)

    
    screen.fill((135, 206, 235))
    pg.display.flip()


    #Pedir datos 2
    ancho = ""

    info2 = Texto("Cual es el ancho?","Black")
    screen.blit(info2.text, (50,50))


    #update the screen
    pg.display.flip()

    running = True
     
    # main loop
    while running:
        # event handling, gets all event from the event queue
        for event in pg.event.get():
            # only do something if the event is of type QUIT
            if event.type == pg.QUIT:
                # change the value to False, to exit the main loop
                running = False
        
            if event.type == pg.KEYDOWN:
                ancho += event.unicode
        
            if event.type == pg.MOUSEBUTTONDOWN:
                if pg.mouse.get_pressed()[0]:
                    screen.fill((135, 206, 235))
                    running = False
        
            info2_surface = Texto(ancho,"Black")
            screen.blit(info2_surface.text, (50,150))

            #update the screen
            pg.display.flip()

    ancho = int(ancho)

    pg.quit()
    return ancho, largo

#Funcion para recibir la informacion del arduino
def arduino_rec_info():
    #!/usr/bin/env python
    # -*- coding: utf-8 -*-
    # lsusb to check device name
    #dmesg | grep "tty" to find port name


    valor = np.array([0,0,0])

    if __name__ == '__main__':
        
        print('Running. Press CTRL-C to exit.')
        with serial.Serial("/dev/ttyACM0", 9600, timeout=1) as arduino:
            time.sleep(0.1) #wait for serial to open
            if arduino.isOpen():
                print("{} connected!".format(arduino.port))
                try:
                    cont = 0
                    while True:
                        #cmd=input("Enter command : ")
                        #arduino.write(cmd.encode())
                        #time.sleep(0.1) #wait for arduino to answer
                        while arduino.inWaiting()==0: pass
                        if  arduino.inWaiting()>0: 
                            
                            answer=arduino.readline()
                            cont = cont + 1


                        valor = np.fromstring(answer, dtype=int, sep=',')
                        print(valor)

                        #Leer 5 veces el sensor para esperar estabilizacion
                        if cont == 5:
                            return valor
                #time.sleep(0.1)
                            #arduino.flushInput() #remove data after reading
                except KeyboardInterrupt:
                    print("KeyboardInterrupt has been caught.")

#Funcion para enviarla informacion al arduino
def arduino_env_info(msg):
    if __name__ == '__main__':
        msg = str(msg)

        print('Running. Press CTRL-C to exit.')
        with serial.Serial("/dev/ttyACM0", 9600, timeout=1) as arduino:
            time.sleep(0.1) #wait for serial to open
            if arduino.isOpen():
                print("{} connected!".format(arduino.port))
                try:
                    while True:
                        #cmd=input("Enter command : ")
                        arduino.write(msg.encode())
                        #time.sleep(0.1) #wait for arduino to answer

                except KeyboardInterrupt:
                    print("KeyboardInterrupt has been caught.")

#Funcion para mapear los valores
def map(x, in_min, in_max, out_min, out_max):
		mapped =  int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)
		return mapped  

#Funcion para actualizar la posicion de cristal en el HMI
def actualizar_pos(ubicacion,posicion_muestras,ancho,largo):
    ubicacion[0] = map(ubicacion[0],0,ancho,10,ancho*30)
    ubicacion[1] = map(ubicacion[1],0,largo,largo*30,20)

    #print(ubicacion)
    # create a surface on screen that has the size of 240 x 180
    screen1 = pg.display.set_mode((500,500))

    screen1.fill((0, 0, 0))

    #dibujar el contorno del mapa

    pg.draw.rect(screen1,(255,255,255),(10,10,ancho*30,largo*30),1)

    pg.draw.rect(screen1,(239,127,26),(ubicacion[0],ubicacion[1],10,10))

    #Dibujar las muestras en el mapa
    for a in posicion_muestras:
        a[0] = map(a[0],0,ancho,10,ancho*30)
        a[1] = map(a[1],0,largo,largo*30,20)

        pg.draw.rect(screen1,(0,0,128),(a[0],a[1],2,2))

    pg.display.flip()

#Funcion para crear los puntos de muestras
def muestras(cantidad,ancho,largo):
    posicion_muestras = np.zeros((cantidad*ancho, 2))

    x=ancho/cantidad 
    valorx=x

    y = largo/cantidad 
    valory = y
    for a in np.arange(posicion_muestras.shape[0]):
        posicion_muestras[a,0] = valorx
        posicion_muestras[a,1] = valory

        valory = valory + y

        if valory > largo:

            valorx = valorx + x
            valory = y

    return posicion_muestras

#Funcion para ir creando el mapa de trabajo
def mapa_trabajo(ubicacion,posicion_muestras,ancho,largo):
    
    # initialize the pygame module
    pg.init()
    
    actualizar_pos(ubicacion,posicion_muestras,ancho,largo)

    running = True
     
    # main loop
    while running:
        # event handling, gets all event from the event queue
        for event in pg.event.get():
            # only do something if the event is of type QUIT
            if event.type == pg.QUIT:
                # change the value to False, to exit the main loop
                running = False

            #update the screen
            pg.display.flip()

#Funcion para convertir la velocidad y la velocidad angular en velocidad de las ruedas
def twistToVel(vel_li,vel_angu):
        w = 0.33        #Distancia entre ruedas
        r = 0.038       #Radio de las ruedas
		
        dx = vel_li     #dx = v lineal
        dr = vel_angu   #dr = v angular
        right = (2.0 * dx + dr * w) / (2*r)
        left = (2.0 * dx - dr * w) / (2*r)
		##!!!!!
		#Se debe mapear las velocidades para enviar la info al arduino
        ##!!!!!

#Funcion para calcular el angulo de error
def angulo(ang,ubicacion, posicion_muestras):
    #ang debe ser la orientacion del robot leida por los nodos de la IMU

    #obj es la matriz homogenea para 2 dimenciones (x,y) teniendo el giro en Z 
    #obj es la postura del robot
    obj = np.array([[-mt.sin(ang),mt.cos(ang),ubicacion[0]],[mt.cos(ang),mt.sin(ang),ubicacion[1]],[0,0,1]])
    obj = np.linalg.inv(obj)

    #Target (seria bueno que se le pregunte al usuario el target de manera manual)
    tar = np.array([[posicion_muestras[0,0]],[posicion_muestras[0,1]],[1]])


    #Calcular la posicion del target con respecto al robot
    pos = obj @ tar

    #print(obj)
    #print("La posicion del target con respecto al robot es: (x: %s , y: %s) " %(pos[0,0],pos[1,0]))

    #Se calcula el angulo de error (angulo que necesitamos rotar)
    ang_gi_rad = mt.atan2(pos[1,0],pos[0,0])
    ang_gi = ang_gi_rad* (180/3.14)

    return ang_gi_rad

#calcula la velocidad lineal y angular del robot
def calculo_velocidades(angulo_gi_rad,ubicacion,posicion_muestras):
    #calcular distancia para la velocidad 
    #print(angulo_gi_rad)
    d = mt.sqrt(((posicion_muestras[0,0]-ubicacion[0])**2)+((posicion_muestras[0,1]-ubicacion[1])**2))

    vel_gi = angulo_gi_rad
    vel_li = d
    
    return vel_li, vel_gi

#Manda el angulo de movimiento al servo usando PCA al servo 15 especificamente
def servo(ang_servo):
    kit.servo[13].angle = ang_servo

#extender el brazo del sensor
def activar():
    gpio.output(4, True)
    gpio.output(17, True)

#retraer el brazo
def desactivar():
    gpio.output(4, False)
    gpio.output(17, False)

#tomar una medicion
def sensar():
    #poner arduino en modo sensar npk
    con_arduino(0,1)

    servo(30)
    time.sleep(3)
    activar()
    time.sleep(3)


    npk = np.array([0,0,0])

    while (((npk[0]+npk[1]+npk[2]) == 0) or (npk[0] == 255) or (npk[1] == 255) or (npk[2] == 255)):
        
        npk = arduino_rec_info()
        

    desactivar()
    time.sleep(3)
    servo(100)
    time.sleep(3)

    return(npk)

#Funcion para controlar el funcionamiento del arduino
def con_arduino(x,y):
    gpio.output(26, x)
    gpio.output(19, y)

#funcion para leer los ultrasonidos
def ultrasonidos():

    #ordenarle al arduino que lea los ultrasonidos
    con_arduino(1,0)
    
    #leer la informacion que manda el arduino
    info_ultrasonido = arduino_rec_info()

    return info_ultrasonido



#Llamado a las funciones

#ancho,largo,posicion_muestras,ubicacion = inicializar()
#mapa_trabajo(ubicacion,posicion_muestras,ancho,largo)
while True:
    ul = ultrasonidos()
    print(ul)
