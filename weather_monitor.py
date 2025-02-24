import requests
import numpy as np
from PIL import Image
from io import BytesIO
import time


def weather_monitor(self):
    # URL эндпоинта, который возвращает изображение
    url = "http://127.0.0.1:8000/get_weather/"
    while True:
        # Выполняем GET-запрос
        print("request weather..")
        response = requests.get(url)

        # Проверяем, что запрос успешен
        if response.status_code == 200:
            # Открываем изображение с помощью PIL
            image = Image.open(BytesIO(response.content))
            # Преобразуем изображение в массив NumPy
            image_np = np.array(image)
            # Получаем размеры изображения
            height, width, _ = image_np.shape
            # Извлекаем цвет из нижнего правого угла
            bottom_right_pixel = image_np[height - 1, width - 1]
            # Цвет в формате BGR (если изображение в формате BGR, как в OpenCV)
            r, g, b = bottom_right_pixel

            print(f"Цвет в нижнем правом углу (BGR): ({b}, {g}, {r})")

            if g > 0 and r == 0 and b == 0:
                print("[Weather Monitor] Погода: зеленая – всё в норме.")
                if self.emergency_event.is_set():
                    self.emergency_event.clear()
                print("[Weather Monitor] Погода нормализовалась. Экстренный режим завершён.")
            elif g > 0 and r > 0 and b == 0:
                print("[Weather Monitor] Погода: желтая – через минуту будет ураган! Инициирую экстренный возврат.")
                if not self.emergency_event.is_set():
                    self.emergency_event.set()
            elif g == 0 and r > 0 and b == 0:
                print("[Weather Monitor] Погода: красная – ураган! Сидим!.")
            else:
                if self.emergency_event.is_set():
                    self.emergency_event.clear()
                print("[Weather Monitor] Погода нормализовалась. Экстренный режим завершён.")
        else:
            print(f"Ошибка: {response.status_code}")

        time.sleep(1)

if __name__ == "__main__":
    weather_monitor()
