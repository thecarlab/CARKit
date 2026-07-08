import numpy as np

# ----------------------------------------
# Exercise 5: Learn a tiny CNN kernel
# ----------------------------------------
# Goal:
# See how a kernel can learn useful numbers from examples.
#
# Important vocabulary:
# - Image pattern: what appears in the input image patch.
# - Kernel: the 3x3 numbers the CNN learns.
# - Target score: the number we want the learned kernel to output.
#
# The target scores come from an example "teacher kernel":
# - center column is positive, because that is the pattern we want
# - left and right columns are negative, because those patterns are not centered
#
# The learning kernel does NOT know the teacher kernel at the beginning.
# It starts with zeros and learns kernel numbers from mistakes.


def score_image(image, kernel):
    """Multiply each image pixel by each kernel value, then add them up."""
    return np.sum(image * kernel)


training_images = [
    np.array([
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
    ]),
    np.array([
        [1, 0, 0],
        [1, 0, 0],
        [1, 0, 0],
    ]),
    np.array([
        [0, 0, 1],
        [0, 0, 1],
        [0, 0, 1],
    ]),
    np.array([
        [0, 0, 0],
        [1, 1, 1],
        [0, 0, 0],
    ]),
]

pattern_names = [
    "center vertical line",
    "left vertical line",
    "right vertical line",
    "horizontal line",
]

teacher_kernel = np.array([
    [-1, 2, -1],
    [-1, 2, -1],
    [-1, 2, -1],
])

# These are target scores, not target kernels.
# We calculate them using the teacher kernel so they are explainable:
# image pattern * teacher kernel -> target score
target_scores = np.array([
    score_image(image, teacher_kernel) for image in training_images
])


def print_grid(name, grid):
    print(name)
    for row in grid:
        print(" ".join(f"{value:5.2f}" for value in row))
    print()


kernel = np.zeros((3, 3))
learning_rate = 0.1
epochs = 200

print_grid("Teacher kernel used to create target scores:", teacher_kernel)

print("Target scores:")
for name, image, target_score in zip(pattern_names, training_images, target_scores):
    print(f"{name:22s} target={target_score:5.2f}")
print()

print_grid("Starting kernel:", kernel)

for epoch in range(epochs):
    total_error = 0

    for image, target_score in zip(training_images, target_scores):
        score = score_image(image, kernel)
        error = target_score - score
        total_error += abs(error)

        # If the score is too low, error is positive and active pixels increase.
        # If the score is too high, error is negative and active pixels decrease.
        kernel = kernel + learning_rate * error * image

    if epoch < 5 or (epoch + 1) % 25 == 0:
        print(f"Epoch {epoch + 1}: total error = {total_error:.2f}")

print()
print_grid("Learned kernel:", kernel)

print("Final test:")
for name, image, target_score in zip(pattern_names, training_images, target_scores):
    score = score_image(image, kernel)
    print(
        f"{name:22s} score={score:5.2f} "
        f"target={target_score:5.2f}"
    )

print()
print("Challenge:")
print("1. Which kernel positions became positive?")
print("2. Which kernel positions became negative?")
print("3. Why does this kernel give the horizontal line a score near 0?")
print("4. Can you change the target scores to learn a horizontal-line kernel?")
