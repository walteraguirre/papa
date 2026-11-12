import pygame
import sys
import time
import random

def main():
    pygame.init()
    pygame.mouse.set_visible(False)
    
    # 1. Configuración de pantalla
    ANCHO = 1920
    ALTO = 1080
    pantalla = pygame.display.set_mode((ANCHO, ALTO), pygame.NOFRAME | pygame.DOUBLEBUF)
    
    # 2. Diseño y dimensiones
    NEGRO = (0, 0, 0)
    COLOR_OJO = (0, 200, 255) # Celeste brillante
    
    OJO_ANCHO = 300
    OJO_ALTO_MAX = 500
    SEPARACION = 400 
    
    # NUEVO: Cuántos píxeles de curvatura tendrán las esquinas (a mayor número, más circular)
    RADIO_ESQUINAS = 80 
    
    # Cálculos para centrar
    CENTRO_X = ANCHO // 2
    CENTRO_Y = ALTO // 2
    
    OJO_IZQ_X = CENTRO_X - SEPARACION // 2 - OJO_ANCHO // 2
    OJO_DER_X = CENTRO_X + SEPARACION // 2 - OJO_ANCHO // 2
    
    # 3. Variables de control
    reloj = pygame.time.Clock()
    estado = "ABIERTO"
    tiempo_ultimo_parpadeo = time.time()
    espera_parpadeo = random.uniform(2.0, 5.0)
    
    altura_actual = OJO_ALTO_MAX
    velocidad_parpadeo = 80 
    
    # 4. Bucle principal
    while True:
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
                
        pantalla.fill(NEGRO)
        
        # MÁQUINA DE ESTADOS MATEMÁTICA
        if estado == "ABIERTO":
            altura_actual = OJO_ALTO_MAX
            if time.time() - tiempo_ultimo_parpadeo > espera_parpadeo:
                estado = "CERRANDO"
                
        elif estado == "CERRANDO":
            altura_actual -= velocidad_parpadeo
            if altura_actual <= 10: 
                altura_actual = 10
                estado = "ABRIENDO"
                
        elif estado == "ABRIENDO":
            altura_actual += velocidad_parpadeo
            if altura_actual >= OJO_ALTO_MAX:
                altura_actual = OJO_ALTO_MAX
                estado = "ABIERTO"
                tiempo_ultimo_parpadeo = time.time()
                espera_parpadeo = random.uniform(2.0, 5.0)
        
        ojo_izq_rect = (OJO_IZQ_X, CENTRO_Y - altura_actual // 2, OJO_ANCHO, altura_actual)
        ojo_der_rect = (OJO_DER_X, CENTRO_Y - altura_actual // 2, OJO_ANCHO, altura_actual)
        
        # NUEVO: Calcular el radio máximo permitido en este fotograma
        # Evita un error (Segfault) de Pygame si la altura es menor que el radio original
        radio_dinamico = min(RADIO_ESQUINAS, altura_actual // 2)
        
        # NUEVO: Dibujar rectángulos con esquinas redondeadas en lugar de elipses
        pygame.draw.rect(pantalla, COLOR_OJO, ojo_izq_rect, border_radius=radio_dinamico)
        pygame.draw.rect(pantalla, COLOR_OJO, ojo_der_rect, border_radius=radio_dinamico)
        
        pygame.display.flip()
        reloj.tick(60) 

if __name__ == "__main__":
    main()
