import os
import cv2
import numpy as np
import random
from tqdm import trange
import random
import torchvision.transforms.v2 as tvt
from directed_blur import KernelBuilder, DirectedBlurFilter
import glob


def overlay_images(base_dir, output_dir, i, gate_img_path, img_class, blur_filter, folder):
    """
    base_dir: Путь к исходной папке с dataset (должны быть подпапки images/train/ и labels/train/)
    output_dir: Путь для сохранения нового dataset (будут созданы подпапки images/train/ и labels/train/)
    overlay_images_paths: Список путей к накладываемым изображениям (3 изображения)
    target_counts: Количество наложений для каждого изображения (например [8000, 8000, 8000])
    """
    background_img_nmb = random.randint(1, 25)

    image_file = cv2.imread(
        f"./backgrounds/{background_img_nmb}.png", cv2.IMREAD_UNCHANGED)
    image_file = cv2.cvtColor(image_file, cv2.COLOR_BGR2BGRA)
    image_file = cv2.resize(image_file, (640, 480))
    gate_img = cv2.imread(gate_img_path, cv2.IMREAD_UNCHANGED)
    gate_img = cv2.cvtColor(gate_img, cv2.COLOR_BGR2BGRA)

    input_path = f"{i:06d}"
    # Читаем соответствующий txt файл
    txt_path = os.path.join(base_dir, 'labels', folder, f"{input_path}.txt")

    with open(txt_path, 'r') as f:
        lines = f.readline()

    x_center, y_center, width, height = map(float, lines.split(" ")[1:5])

    # Конвертируем нормализованные координаты в пиксельные
    img_h, img_w = image_file.shape[:2]
    x_center_px = int(x_center * img_w)
    y_center_px = int(y_center * img_h)
    width_px = int(width * img_w)
    height_px = int(height * img_h)

    # Вычисляем координаты верхнего левого угла
    x1 = x_center_px - width_px // 2
    y1 = y_center_px - height_px // 2

    # Получаем накладываемое изображение и изменяем его размер
    gate_img_resized = cv2.resize(gate_img, (width_px, height_px))

    if gate_img_resized.shape[2] == 4:
        # Разделяем каналы
        overlay_bgr = gate_img_resized[:, :, :3]
        overlay_alpha = gate_img_resized[:, :, 3] / 255.0
        # Вычисляем область для наложения
        y2 = y1 + height_px
        x2 = x1 + width_px

        # Проверяем границы
        if x1 >= 0 and y1 >= 0 and x2 <= img_w and y2 <= img_h:
            # Наложение с учетом альфа-канала
            for c in range(3):
                image_file[y1:y2, x1:x2, c] = (
                    1.0 - overlay_alpha) * image_file[y1:y2, x1:x2, c] + overlay_alpha * overlay_bgr[:, :, c]

            new_lines = []
            parts = lines.strip().split()
            parts[0] = str(img_class)  # Меняем class_id
            new_lines.append(" ".join(parts) + "\n")
            # Сохраняем измененное изображение
            image_file = blur_filter(image_file)
            output_img_path = os.path.join(
                output_dir, 'images', folder, f"{input_path}.png")
            cv2.imwrite(output_img_path, image_file)

            # Сохраняем измененный txt файл
            output_txt_path = os.path.join(
                output_dir, 'labels', folder, f"{input_path}.txt")
            with open(output_txt_path, 'w') as f:
                f.writelines(new_lines)

    else:
        y2 = y1 + height_px
        x2 = x1 + width_px
        if x1 >= 0 and y1 >= 0 and x2 <= img_w and y2 <= img_h:
            image_file[y1:y2, x1:x2] = gate_img_resized

            new_lines = []
            parts = lines.strip().split()
            parts[0] = str(img_class)  # Меняем class_id
            new_lines.append(" ".join(parts) + "\n")
            # Сохраняем измененное изображение
            image_file = blur_filter(image_file)
            output_img_path = os.path.join(
                output_dir, 'images', folder, f"{input_path}.png")
            cv2.imwrite(output_img_path, image_file)

            # Сохраняем измененный txt файл
            output_txt_path = os.path.join(
                output_dir, 'labels', folder, f"{input_path}.txt")
            with open(output_txt_path, 'w') as f:
                f.writelines(new_lines)


base_dir = './dataset5'
output_dir = './dataset'
for folder in ['images', 'labels']:
    for subfolder in ['train', 'val']:
        os.makedirs(os.path.join(output_dir, folder, subfolder), exist_ok=True)


gate_images = glob.glob("./gates_photos/*.png")

kernel_builder = KernelBuilder()
kernel_size = np.random.choice(np.arange(3, 10, 2, dtype=int), size=1)[0]
directed_blur_filter = tvt.GaussianBlur(kernel_size=kernel_size)

for i in trange(0, 27429, ncols=80, desc="Total train"):
    if i % 50 == 0:
        kernel_size = np.random.choice(
            np.arange(3, 10, 2, dtype=int), size=1)[0]
        directed_blur_filter = tvt.GaussianBlur(kernel_size=kernel_size)

    class_id = random.randint(0, len(gate_images) - 1)
    gate_image = gate_images[class_id]
    overlay_images(base_dir, output_dir, i, gate_image,
                   class_id, directed_blur_filter, "train")


for i in trange(27429, 35428, ncols=80, desc="Total val"):
    if i % 50 == 0:
        kernel_size = np.random.choice(
            np.arange(3, 10, 2, dtype=int), size=1)[0]
        directed_blur_filter = tvt.GaussianBlur(kernel_size=kernel_size)
    class_id = random.randint(0, len(gate_images) - 1)
    gate_image = gate_images[class_id]
    input_path = f"{i:06d}"
    overlay_images(base_dir, output_dir, i, gate_image,
                   class_id, directed_blur_filter, "val")
