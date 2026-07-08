import cv2

img = cv2.imread("Clear_stop_sign.jpg")

print(type(img))
print(img.shape)
print(img[100, 200])

# TODO: convert the image to grayscale and print the shape of the grayscale image


cv2.imwrite("exercise_1.jpg", img)

