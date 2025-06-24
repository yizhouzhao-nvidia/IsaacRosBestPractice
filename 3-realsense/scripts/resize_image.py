import cv2

# Read the image from file
img = cv2.imread('/home/yizhou/Downloads/1.jpg')

frame_resized = cv2.resize(img, (640, 480))

# Check if the image was loaded successfully
if img is None:
    print("Failed to load image!")
else:
    print("Image loaded. Shape:", img.shape)


cv2.imwrite("captured_image.jpg", frame_resized)