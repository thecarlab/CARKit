import cv2

img = cv2.imread("Clear_stop_sign.jpg")
cv2.imwrite("edited_stop_sign.jpg", img)
img[100:200, 100:200] = [225, 225, 255]

# TODO: Draw different colors at different positions other than the already drawn red square


# TODO: change the pixel of origin (0, 0) to white


# TODO: Draw a red 100x100 square at the center of the image
center_x = img.shape[1] // 2
center_y = img.shape[0] // 2
range = 100



cv2.imwrite("exercise_2.jpg", img)