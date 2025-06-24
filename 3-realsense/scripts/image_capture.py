import cv2

# Open the default webcam (device 0)
cap = cv2.VideoCapture(4)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Capture a single frame
for _ in range(20):
    ret, frame = cap.read()


if not ret:
    print("Error: Could not read frame from webcam.")
    cap.release()
    exit()

# Resize to 640x480
frame_resized = cv2.resize(frame, (640, 480))

# Convert BGR to RGB (if you want to check in an RGB viewer)
frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)

# Save as JPEG file
output_filename = "captured_image.jpg"
cv2.imwrite(output_filename, frame_resized)

print(f"Image saved as {output_filename}")

# Release the webcam
cap.release()
