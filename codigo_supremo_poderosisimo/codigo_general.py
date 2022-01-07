#importar la libreria
import pygame as pg
import serial,time
import numpy as np

#Funciones

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
                    while True:
                        #cmd=input("Enter command : ")
                        #arduino.write(cmd.encode())
                        #time.sleep(0.1) #wait for arduino to answer
                        while arduino.inWaiting()==0: pass
                        if  arduino.inWaiting()>0: 
                            answer=arduino.readline()

                    valor = np.fromstring(answer, dtype=int, sep=',')


                    print(valor)
                #time.sleep(0.1)
                            #arduino.flushInput() #remove data after reading
                except KeyboardInterrupt:
                    print("KeyboardInterrupt has been caught.")

#Funcion para mapear los valores
def map(x, in_min, in_max, out_min, out_max):
		mapped =  int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)
		return mapped  

#Funcion para actualizar la posicion de cristal en el HMI
def actualizar_pos(ubicacion,posicion_muestras):
    ubicacion[0] = map(ubicacion[0],0,ancho,10,ancho*30)
    ubicacion[1] = map(ubicacion[1],0,largo,largo*30,20)

    print(ubicacion)
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

    
    print(posicion_muestras)

#Funcion para ir creando el mapa de trabajo
def mapa_trabajo(ancho,largo,ubicacion):
    
    # initialize the pygame module
    pg.init()
    
    posicion_muestras = muestras(10,ancho,largo)

    actualizar_pos(ubicacion,posicion_muestras)

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

#Llamado a las funciones

#ubicacion inicial del robot
ubicacion = np.array([0,0])
#dimensiones del mapa
#ancho, largo = HMI()

ancho = 10
largo = 10
mapa_trabajo(ancho,largo,ubicacion)