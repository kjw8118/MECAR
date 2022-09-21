import cv2

cap = cv2.VideoCapture(0,cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while cv2.waitKey(1) != 27:
    ret, frame = cap.read()
    cv2.imshow("View", frame)

cap.release()
cv2.destroyAllWindows()