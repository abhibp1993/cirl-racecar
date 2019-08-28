import numpy as np
import cv2

training_img = cv2.imread('test.png', 0) # trainImage

# Initiate detector
detector = cv2.ORB_create()

# Find the keypoints and descriptors
training_kp, training_des = detector.detectAndCompute(training_img, None)

cap = cv2.VideoCapture(0)

while True:
	# Read frame from camera and find keypoints and detectors
	ret, frame = cap.read()
	frame_kp, frame_des = detector.detectAndCompute(frame, None)

	# create BFMatcher object
	bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

	# Match descriptors.
	matches = bf.match(frame_des, training_des)

	# Sort them in the order of their distance.
	matches = sorted(matches, key = lambda x:x.distance)

	# Draw first 10 matches.
	img3 = cv2.drawMatches(frame, frame_kp, training_img, training_kp, matches[:10], None, flags=2)

	#plt.imshow(img3),plt.show()
	cv2.imshow('Output', img3)


	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
