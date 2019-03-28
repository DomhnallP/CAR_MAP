import numpy as np
import cv2

cap = cv2.VideoCapture("/usr/local/home/u180107/FYP/Videos/AW0006.mp4")

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 10.0, (1280,720))

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:
        frame = frame[720:1440, 0:1280]

        # write the flipped frame
        out.write(frame)

        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()