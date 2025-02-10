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

    import numpy as np

class Drone:
    def __init__(self, horizonalSpeed, verticalSpeed, horizontalAccel, verticalAccel, maxRadius=20):
        self.horizontalSpeed = horizonalSpeed
        self.verticalSpeed = verticalSpeed
        self.horizontalAccel = horizontalAccel
        self.verticalAccel = verticalAccel
        self.maxRadius = maxRadius

    def GetMaxSpeed(self, radius, deltaBearing, deltaAttack):
        # Calculate maximum radius based on deltaBearing
        maxRadius = self.maxRadius * (180 - np.abs(deltaBearing)) / 180
        
        # Adjust horizontal speed based on radius
        if radius < maxRadius:
            horizontalFactor = np.power(radius / self.maxRadius, 1/3)
        else:
            horizontalFactor = np.power(maxRadius / self.maxRadius, 1/3)

        # Handle deltaAttack differently for positive and negative values
        if deltaAttack >= 0:
            # Positive deltaAttack increases drag (pitching up)
            dragFactor = 1 - (np.abs(deltaAttack) / 90) * 0.3  # Up to 30% speed loss at 90°
        else:
            # Negative deltaAttack reduces drag (pitching down), boosting horizontal speed
            dragFactor = 1 + (np.abs(deltaAttack) / 90) * 0.2  # Up to 20% speed gain at -90°

        horizontalSpeedAdjusted = horizontalFactor * dragFactor * self.horizontalSpeed

        # Adjust vertical speed
        verticalFactor = 1 - (np.abs(deltaBearing) / 180) * 0.05  # Up to 20% vertical speed loss at sharp turns
        
        if deltaAttack >= 0:
            # Positive deltaAttack increases vertical speed (lifting up)
            climbFactor = 1 + (np.abs(deltaAttack) / 180)   # Up to 100% vertical speed gain at 90°
        else:
            # Negative deltaAttack reduces vertical speed (descending)
            climbFactor = 1 - (np.abs(deltaAttack) / 180)   # Up to 30% vertical speed loss at -90°

        verticalSpeedAdjusted = verticalFactor * climbFactor * self.verticalSpeed

        return horizontalSpeedAdjusted, verticalSpeedAdjusted


        