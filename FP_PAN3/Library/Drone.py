# from Library.LatLonProjection import *
# import numpy as np

# class Drone:
#     def __init__(self, horizonalSpeed, verticalSpeed, horizontalAccel, verticalAccel, maxRadius=20):
#         self.horizontalSpeed = horizonalSpeed
#         self.verticalSpeed = verticalSpeed
#         self.horizontalAccel = horizontalAccel
#         self.verticalAccel = verticalAccel
#         self.maxRadius = maxRadius

#     def GetMaxSpeed(self, radius, deltaBearing):
#         # Calcular el radio máximo ajustado por el delta de dirección
#         maxRadius = self.maxRadius * (180 - np.abs(deltaBearing)) / 180
        
#         # Suavizar la reducción de la velocidad horizontal usando una raíz cuadrada
#         if radius < maxRadius:
#             factor = np.sqrt(radius / self.maxRadius)  # Raíz cuadrada para suavizar la reducción
#             return factor * self.horizontalSpeed, self.verticalSpeed
#         else:
#             factor = np.sqrt(maxRadius / self.maxRadius)
#             return factor * self.horizontalSpeed, self.verticalSpeed
        
import numpy as np

class Drone:
    def __init__(self, horizonalSpeed, verticalSpeed, horizontalAccel, verticalAccel, maxRadius=20):
        self.horizontalSpeed = horizonalSpeed
        self.verticalSpeed = verticalSpeed
        self.horizontalAccel = horizontalAccel
        self.verticalAccel = verticalAccel
        self.maxRadius = maxRadius

    def GetMaxSpeed(self, radius, deltaBearing, deltaAttack):
        # Calcular el radio máximo ajustado por el delta de dirección
        maxRadius = self.maxRadius * (180 - np.abs(deltaBearing)) / 180
        
        # Factor de ajuste para la velocidad horizontal basado en el radio y deltaAttack
        if radius < maxRadius:
            horizontalFactor = np.power(radius / self.maxRadius, 1/3)
        else:
            horizontalFactor = np.power(maxRadius / self.maxRadius, 1/3)

        # Modificar el horizontalFactor basado en deltaAttack: mayor deltaAttack -> más resistencia (menos velocidad horizontal)
        dragFactor = 1 - (np.abs(deltaAttack) / 90) * 0.3  # up to 30% horizontal speed loss at 90 degrees of attack
        horizontalSpeedAdjusted = horizontalFactor * dragFactor * self.horizontalSpeed

        # Ajustar la velocidad vertical según deltaBearing y deltaAttack
        # As sharper turns or larger deltaAttack angles occur, vertical speed might decrease
        verticalFactor = 1 - (np.abs(deltaBearing) / 180) * 0.2  # Lose up to 20% vertical speed at sharp turns
        liftFactor = 1 + (np.abs(deltaAttack) / 90) * 0.4  # Gain up to 40% vertical speed at 90 degrees of attack

        verticalSpeedAdjusted = verticalFactor * liftFactor * self.verticalSpeed

        return horizontalSpeedAdjusted, verticalSpeedAdjusted

