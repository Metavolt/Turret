import cv2
import imutils
import sys
import time
import random
import simpleaudio.functionchecks as fc
import simpleaudio
from adafruit_servokit import ServoKit
from subprocess import call

print('Initializing turret...')

kit = ServoKit(channels=16)
cascPath = "/home/pi/Turret/haarcascade_frontalface_default.xml"
faceCascade = cv2.CascadeClassifier(cascPath)

# You may need to adjust these set points depending on how you physically mounted your servo
SERVO_PAN_CENTER = 85
SERVO_PAN_MIN = 15
SERVO_PAN_MAX = 155
SERVO_TILT_CENTER = 115
SERVO_TILT_MIN = 75
SERVO_TILT_MAX = 145
FRAME_X_CENTER = 320
FRAME_Y_CENTER = 240
THRESHOLD = 26
PAN_CHANNEL = 14
TILT_CHANNEL = 15
startup_clips = ['initialized', 'hello', 'system_nominal']
detection_clips = ['target_detect', 'target_lock', 'got_you']
looking_clips = ['staring', 'hold_still', 'see_you', 'ugly']
idle_clips = ['scanning', 'still_there', 'job_sucks']

#fc.LeftRightCheck.run() # Quick way to test if audio is working at all

def play_clip(clip_description):
	# TODO add some error handling...
	voice_clip = simpleaudio.WaveObject.from_wave_file('/home/pi/Turret/Voice/tur_' + clip_description + '.wav')
	voice_clip.play()

def map(val, in_min, in_max, out_min, out_max):
	return(val - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def constrain(val, min, max):
	if val < min:
		return min
	elif val > max:
		return max
	else:
		return val

def set_servos(pan_pos, tilt_pos):
	if pan_pos > 0: kit.servo[PAN_CHANNEL].angle = pan_pos
	if tilt_pos> 0: kit.servo[TILT_CHANNEL].angle = tilt_pos

servo_pan_current = SERVO_PAN_CENTER
servo_pan_delta = 15
servo_tilt_current = SERVO_TILT_CENTER
time_face_detected = 0
is_face_detected = False
time_face_lost = 0
is_face_lost = True
target_insulted = False
idle_clip_played = False
scanning = False
last_scan_time = 0

set_servos(SERVO_PAN_CENTER, -1)
time.sleep(0.5)
set_servos(-1, SERVO_TILT_CENTER)
time.sleep(0.5)

video_capture = cv2.VideoCapture(0)
play_clip(random.choice(startup_clips))
print('initialized')

while True:

	if not video_capture.isOpened():
		print('Camera Load Error')
		sleep(5)
		pass

	return_code, frame = video_capture.read()
	rot_frame = imutils.rotate(frame, 180) # The camera is upside down :)
	grayscale_image = cv2.cvtColor(rot_frame, cv2.COLOR_BGR2GRAY)
	faces = faceCascade.detectMultiScale(grayscale_image, scaleFactor=1.1, minNeighbors=5, minSize=(32, 32))
	#print('Faces detected:{}'.format(len(faces)))
	time_now = time.thread_time()

	# Allow some flexibilty so the face can be found immediately, but not counted as lost immediately
	if len(faces) and not is_face_detected:
		is_face_detected = True
		target_insulted = False
		idle_clip_played = False
		scanning = False
		time_face_detected = time_now
		print('tracking now')
		play_clip(random.choice(detection_clips))

	# We don't want to react if a face is only briefly noticed
	if time_now - time_face_detected > 12.0 and not target_insulted and len(faces):
		play_clip(random.choice(looking_clips))
		target_insulted = True

	# We need to keep track of when the face was actually lost before we say for sure
	if len(faces) < 1 and is_face_detected and not is_face_lost:
		time_face_lost = time_now
		is_face_lost = True
		print ('target lost at {0:3f} seconds'.format(time_face_lost))
		# add "search" routine here

	# We also need to keep track of when we reacquire a face
	if len(faces) and is_face_lost:
		is_face_lost = False
		time_face_detected = time_now
		print ('target found in {0:3f} seconds'.format(time_now - time_face_lost))

	# Now we can adjust how long before a face is considered lost
	if time_now - time_face_lost > 3.0 and is_face_lost:
		print('Current clock: {0:.3f}'.format(time.thread_time()))
		is_face_detected = False
		is_face_lost = False
		print('Face tracked for roughly:{0:.2f} seconds \n'.format(time_now - time_face_detected))
		#set_servos(SERVO_PAN_CENTER, SERVO_TILT_CENTER)

	if time_now - time_face_lost > 10.0 and not is_face_detected and not idle_clip_played:
		play_clip(random.choice(idle_clips))
		idle_clip_played = True
		scanning = True
		set_servos(SERVO_PAN_CENTER, servo_pan_current)

	# Only track the largest face in frame
	face_widths = []
	largest_face_index = 0

	for face in faces:
		face_widths.append(face[2]) # Faces are lists containing [x, y, width, height]

	if len(faces) and not scanning:

		largest_face_index = face_widths.index(max(face_widths))
		face_widths.clear()
		(x, y, width, height) = faces[largest_face_index]
		#print('face 0 x:{}, y:{}'.format(faces[][0], faces[0][1]))
		# X and Y of a face are anchored in the top left, so we need to find the center of the bounding rect
		face_center_x = width // 2 + x
		face_center_y = height // 2 + y
		#print('X dist:{}; Y dist:{}'.format(FRAME_X_CENTER-face_center_x, FRAME_Y_CENTER-face_center_y))
		#cv2.circle(rot_frame, (face_center_x, face_center_y), 2, (0, 0, 255), 1)
		#cv2.circle(rot_frame, (face_center_x, face_center_y), width//2, (255, 255, 0), 1)

		if face_center_x > FRAME_X_CENTER + THRESHOLD:
			servo_pan_current -= 1
		elif face_center_x < FRAME_X_CENTER - THRESHOLD:
			servo_pan_current += 1
		if face_center_y > FRAME_Y_CENTER + THRESHOLD:
			servo_tilt_current += 1
		elif face_center_y < FRAME_Y_CENTER - THRESHOLD:
			servo_tilt_current -= 1

		servo_pan_current = constrain(servo_pan_current, SERVO_PAN_MIN, SERVO_PAN_MAX)
		servo_tilt_current = constrain(servo_tilt_current, SERVO_TILT_MIN, SERVO_TILT_MAX)
		set_servos(servo_pan_current, servo_tilt_current)

	if scanning:

		if time_now - last_scan_time > 2.5:
			servo_pan_current += servo_pan_delta

			if servo_pan_current >= SERVO_PAN_MAX or servo_pan_current <= SERVO_PAN_MIN:
				servo_pan_delta *= -1
			set_servos(servo_pan_current, servo_tilt_current)
			last_scan_time = time_now

		# There's no user interface per se, so just turn off eventually
		if time_now - time_face_lost > 60.0 and not is_face_detected:
			for pos in range(servo_tilt_current, SERVO_TILT_MAX):
				set_servos(-1, pos)
				time.sleep(0.1)
			call('sudo shutdown -h now', shell=True)

	if cv2.waitKey(1) & 0xFF ==ord('q'):
		break

	#cv2.imshow('Video', rot_frame)

video_capture.release()
cv2.destroyAllWindows()

