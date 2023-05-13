import cv2
import numpy as np

class KalmanFilter:
    kf = cv2.KalmanFilter(3, 3)
    kf.transitionMatrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], np.float32)
    kf.measurementMatrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], np.float32)
    


    def predict(self, coordX, coordY, z):
        ''' This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)], [np.float32(z)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        x, y, z = int(predicted[0]), int(predicted[1]), int(predicted[2])
        return x, y, z
