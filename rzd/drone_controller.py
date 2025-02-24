from typing import List, Tuple, Dict, Optional
from pion import Pion
from pyzbar.pyzbar import decode
import math
import threading
from typing import Union, Optional
# Импорт необходимых функций из модуля pion.functions
from pion.functions import vector_reached, update_array
import cv2
import numpy as np
from .drone_cv import *
import time
import requests

# Для РТС
import multiprocessing
from omegabot_poligon77 import *  # Необходимо утсановить пакет из README
from RTS_code import *


def detect_object(drone: Pion, detect_code: str) -> None:
    """
    Функция вызывается, если вы задетектировали detect_code при сканировании или при доставке
    
    :param drone: объект дрона
    :type drone: Pion

    :param detect_code: Строка со значением из qr кода
    :type detect_code: str

    :rtype: None
    """
    print("detect_object(), ip: ", drone.ip, "detect_code: ", detect_code)
    try:
        requests.get("http://10.1.100.6:31556/detect_object",
                     params={
                         "object": f"{detect_code.replace(" ", "_")}",
                         "host": drone.ip[-3:]
                     }).text
    except:
        print("Геймкор выключен")


def get_box(drone: Pion) -> None:
    """
    Функция вызывается, если вы сели на qr код (для дрона доставщика)
    
    :param drone: объект дрона
    :type drone: Pion

    :rtype: None
    """
    print("get_box(), ip: ", drone.ip)
    try:
        requests.get("http://10.1.100.6:31556/get_box",
                     params={"host": drone.ip[-3:]}).text
    except:
        print("Геймкор выключен")


def drop_box(drone: Pion) -> None:
    """
    Функция вызывается, если вы сбрасываете груз
    
    :param drone: объект дрона
    :type drone: Pion

    :rtype: None
    """
    print("drop_box(), ip: ", drone.ip)
    try:
        requests.get("http://10.1.100.6:31556/drop_object",
                     params={"host": drone.ip[-3:]}).text
    except:
        print("Геймкор выключен")


def calculate_shift_global(points: np.ndarray,
                           frame_center: Tuple[int, int],
                           yaw: float,
                           altitude: float) -> List[float]:
    """
    Вычисляет смещение QR-кода относительно центра кадра.

    :param points: Координаты вершин найденного QR-кода.
    :type points: np.ndarray
    :param frame_center: Центр кадра (ширина, высота).
    :type frame_center: Tuple[int, int]
    :param yaw: Текущий угол поворота дрона (в радианах).
    :type yaw: float
    :param altitude: Текущая высота дрона.
    :type altitude: float
    :return: Список с корректированными смещениями [x, y].
    :rtype: List[float]
    """
    qr_center = np.mean(points, axis=0)
    shift_x_px = qr_center[0] - frame_center[0]
    shift_y_px = qr_center[1] - frame_center[1]
    shift_x_m = (shift_x_px * altitude) / 700  # 700 – фокальная длина
    shift_y_m = (shift_y_px * altitude) / 700
    # Преобразование с учетом угла yaw:
    corrected_x = shift_x_m * math.cos(yaw) - shift_y_m * math.sin(yaw)
    corrected_y = shift_x_m * math.sin(yaw) + shift_y_m * math.cos(yaw)
    return [corrected_x, corrected_y]


def detect_qr_global_from_frame(drone: Pion,
                                frame: np.ndarray,
                                finished_targets: List[str],
                                frame_center: Tuple[int, int],
                                coordinates_or_error: bool = True
                                ) -> Tuple[Dict[str, np.ndarray], np.ndarray]:
    """
    Аналог функции detect_qr_global, но получает уже готовый кадр.
    """
    key_errors: Dict[str, np.ndarray] = {}
    data = decode(frame)
    if data:
        for item in data:
            decoded_key = item[0].decode()
            if decoded_key not in finished_targets:
                drone.led_control(255, 0, 255, 0)
                points = np.array(item[3])
                shift = calculate_shift_global(points, frame_center, drone.yaw, drone.position[2])
                if shift:
                    if coordinates_or_error:
                        error = np.array([-shift[0], shift[1], 0, 0])
                    else:
                        error = np.array([-shift[0] + drone.xyz[0],
                                          shift[1] + drone.xyz[1], 0, 0])
                    key_errors[decoded_key] = error
                    detect_object(drone=drone, detect_code=decoded_key)
                    print(f"Обнаружен: {decoded_key} = {error}")
                drone.led_control(255, 0, 0, 0)
    return key_errors, frame


def detect_qr_global(drone: Pion,
                     cap: cv2.VideoCapture,
                     finished_targets: List[str],
                     frame_center: Tuple[int, int],
                     coordinates_or_error: bool = True
                     ) -> Tuple[Dict[str, np.ndarray], np.ndarray]:
    """
    Считывает кадр из видеопотока, ищет QR-коды и вычисляет error-вектор. Если coordinates_or_error=True,
    возвращается вектор смещения относительно дрона; иначе вектор суммируется с координатами дрона для получения
    глобальных координат.

    :param drone: Объект дрона.
    :type drone: Pion
    :param cap: Объект VideoCapture.
    :type cap: cv2.VideoCapture
    :param finished_targets: Список уже обработанных QR-кодов.
    :type finished_targets: List[str]
    :param frame_center: Центр кадра (ширина, высота).
    :type frame_center: Tuple[int, int]
    :param coordinates_or_error: Флаг выбора типа возвращаемых координат.
    :type coordinates_or_error: bool
    :return: Кортеж (словарь обнаруженных QR, считанный кадр).
    :rtype: Tuple[Dict[str, np.ndarray], np.ndarray]
    """
    with drone._handler_lock:
        ret, frame = cap.read()
        if not ret:
            print("Не удалось получить кадр")
            return {}, frame
    key_errors: Dict[str, np.ndarray] = {}
    data = decode(frame)
    if data:
        for item in data:
            decoded_key = item[0].decode()
            if decoded_key not in finished_targets:
                drone.led_control(255, 0, 255, 0)
                points = np.array(item[3])
                shift = calculate_shift_global(points, frame_center, drone.yaw, drone.position[2])
                if shift:
                    if coordinates_or_error:
                        error = np.array([-shift[0], shift[1], 0, 0])
                    else:
                        error = np.array([-shift[0] + drone.xyz[0],
                                          shift[1] + drone.xyz[1], 0, 0])
                    key_errors[decoded_key] = error
                    print(f"Обнаружен: {decoded_key} = {error}")
                drone.led_control(255, 0, 0, 0)
    return key_errors, frame


def move_to_target(drone: Pion,
                   camera: Union[RTSPCamera, SocketCamera, BaseCamera],
                   finished_targets: List[str],
                   show: bool = False,
                   key: str = '4',
                   scaling_factor: float = 0.5,
                   threshold: float = 0.05,
                   time_break: float = float('inf')
                   ) -> Tuple[List[str], np.ndarray]:
    """
    Корректирует позицию дрона с помощью видеопотока до достижения заданной точности для указанного QR-кода.

    :param drone: Объект дрона.
    :type drone: Pion
    :param  camera: Объект Union[RTSPCamera, SocketCamera]
    :type camera: Union[RTSPCamera, SocketCamera]
    :param finished_targets: Список уже обработанных QR-кодов
    :type finished_targets: List[str]
    :param show: Флаг отображения видеопотока
    :type show: bool
    :param key: Ключ (название) QR-кода, по которому корректируется позиция
    :type key: str
    :param scaling_factor: Коэффициент масштабирования корректирующей скорости
    :type scaling_factor: float
    :param threshold: Порог точности для завершения корректировки (в метрах)
    :type threshold: float
    :param time_break: Максимальное время работы корректировки
    :type time_break: float
    :return: Кортеж (обновлённый список finished_targets, конечные координаты дрона)
    :rtype: Tuple[List[str], np.ndarray]
    """
    errors = np.zeros((10, 4))
    flag_reach_zero_error = False
    drone.speed_flag = False
    t_0 = time.time()
    while not flag_reach_zero_error:
        if time.time() - t_0 > time_break:
            break
        frame = camera.get_cv_frame()
        if frame is None:
            continue
        # Вычисляем центр кадра на основе полученных размеров
        height, width, _ = frame.shape
        frame_center = (width // 2, height // 2)
        # Используем функцию, которая принимает уже полученный кадр
        key_errors, frame = detect_qr_global_from_frame(drone, frame, finished_targets, frame_center)
        print("key_errors =", key_errors)
        if key in key_errors:
            if show:
                cv2.imshow('Delivery stream', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            errors = update_array(errors, key_errors[key])
            flag_reach_zero_error = vector_reached(np.zeros(4), errors, accuracy=threshold)
            adjusted_speed = key_errors[key] * scaling_factor
            print("Отправляем скорость:", *adjusted_speed, f"\nxyz: {drone.xyz}")
            drone.send_speed(*adjusted_speed)
    drone.t_speed = np.zeros(4)
    finished_targets.append(key)
    final_coordinate = drone.xyz.copy()
    return finished_targets, final_coordinate


class DroneScanner:
    """
    Класс для сканирования окружающей среды дроном с целью обнаружения QR-кодов.
    После выполнения миссии сканирования результатом является словарь обнаруженных QR-кодов,
    где для каждого кода усреднены координаты (на основе накопленных error-векторов).
    """
    FOCAL_LENGTH: int = 700

    def __init__(self, drone: Pion, base_coords: np.ndarray, scan_points: np.ndarray,
                 camera: BaseCamera, show: bool = False) -> None:
        """
        :param drone: Объект дрона.
        :param base_coords: Координаты базы (точка возврата).
        :param scan_points: Массив точек для обхода (каждая точка – кортеж (x, y, z, yaw)).
        :param camera: Экземпляр камеры, реализующий BaseCamera.
        :param show: Флаг отображения видеопотока.
        """
        self.show = show
        self.drone: Pion = drone
        self.base_coords: np.ndarray = base_coords
        self.scan_points: np.ndarray = scan_points
        self.unique_points: Dict[str, List[np.ndarray]] = {}  # Накопление QR-кодов
        self.camera = camera  # Используем универсальную камеру вместо VideoCapture

    def initialize_drone(self) -> None:
        """
        Инициализирует дрона: выполняет арминг, взлёт и устанавливает режим скорости.
        """
        print("initialize_drone")
        self.smart_take_off()
        time.sleep(1)

    def smart_take_off(self) -> None:
        """
        Функция для принудительного взлёта.
        """
        print("Smart take off is beginning")
        while self.drone.xyz[2] < 0.3:
            time.sleep(1)
            self.drone.arm()
            time.sleep(0.5)
            self.drone.takeoff()
        time.sleep(3)
        print("Smart take off is ending")

    def detect_qr(self, finished_targets: Optional[List[str]] = None,
                  coordinates_or_error: bool = True) -> Tuple[
        Dict[str, np.ndarray], Optional[np.ndarray], Tuple[int, int]]:
        """
        Получает кадр с камеры, вычисляет центр и ищет QR-коды.
        """
        frame = self.camera.get_cv_frame()
        if frame is None:
            print("Не удалось получить кадр")
            return {}, None, (0, 0)
        height, width, _ = frame.shape
        frame_center = (width // 2, height // 2)
        key_errors, frame = detect_qr_global_from_frame(self.drone, frame, finished_targets or [], frame_center,
                                                        coordinates_or_error)
        return key_errors, frame, frame_center

    def process_mission_point(self,
                              target_point: Tuple[float, float],
                              show: bool = False,
                              emergency_event: threading.Event = None
                              ) -> None:
        """
        Перемещает дрона-сканер к заданной точке и собирает обнаруженные QR-коды.
        При возникновении экстренной ситуации дрон возвращается на базу, ждёт нормализации и затем возобновляет миссию.
        """
        print(f"process_mission_point {target_point}")
        finished_targets = list(self.unique_points.keys())
        self.drone.speed_flag = False
        while True:
            # Обработка экстренной ситуации
            if emergency_event and emergency_event.is_set():
                current_position = self.drone.xyz.copy()
                print("[Emergency] Обнаружена экстренная ситуация! Приостанавливаю миссию.")
                self.return_to_base()
                while emergency_event.is_set():
                    print("[Emergency] Жду нормализации погоды...")
                    time.sleep(1)
                print("[Emergency] Погода нормализовалась. Возобновляю миссию.")
                self.smart_take_off()
                self.drone.goto_from_outside(*current_position, 0)

            if self.drone.xyz[1] > target_point[1]:
                break

            vector_speed = np.array(target_point) - self.drone.xyz[:2]
            vector_length = np.linalg.norm(vector_speed)
            if vector_length > 0:
                vector_speed = (vector_speed / vector_length) * 0.1
                self.drone.send_speed(vector_speed[0], vector_speed[1], 0, 0)

            key_errors, frame, _ = self.detect_qr(finished_targets, coordinates_or_error=False)
            for key, error in key_errors.items():
                self.unique_points.setdefault(key, []).append(error)
                finished_targets.append(key)

            if self.show and frame is not None:
                cv2.imshow(f'Drone Scanner {self.drone.ip}', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        self.drone.speed_flag = False

    def execute_scan(self, emergency_event: threading.Event) -> Dict[str, np.ndarray]:
        print("scaner: start")
        """
        Выполняет миссию сканирования: дрон перемещается по заданным точкам, собирает QR-коды и возвращается на базу.
        """
        print(f"scaner: go to start {self.scan_points[0]}")
        x, y, z, a = self.scan_points[0]
        #self.drone.goto_from_outside(x, y, z, 0, 0.5)
        self.process_mission_point((x, y), False, emergency_event)
        self.drone.speed_flag = False
        for point in self.scan_points[1:]:
            print(f"scaner: go to {point}")
            self.drone.speed_flag = True
            x, y, z, a = point
            #self.drone.goto_from_outside(x, y, z, 0, 0.5)
            #self.drone.speed_flag = False
            #time.sleep(1)
            self.process_mission_point((x, y), False, emergency_event)
        return dict

    def return_to_base(self) -> None:
        """
        Возвращает дрона-сканер на базу.
        """
        print(f"\n\nReturn to base: {self.base_coords}\n\n")
        self.drone.set_v()
        self.drone.goto_from_outside(*self.base_coords[0:2], 0.7, 0)
        self.drone.speed_flag = False
        self.drone.land()
        time.sleep(20)

    @property
    def scanned_qr(self) -> Dict[str, np.ndarray]:
        """
        Возвращает усреднённые координаты всех обнаруженных QR-кодов.

        :return: Словарь, где ключ – код, значение – усреднённая координата.
        :rtype: Dict[str, np.ndarray]
        """
        # Здесь вы должны написать код сами и вернуть Dict
        pass


# ------------------ Класс DroneDeliverer ------------------

class DroneDeliverer:
    """
    Класс для доставки. Дрон-доставщик получает словарь QR-кодов с координатами,
    после чего для каждой найденной цели выполняется корректировка позиции с помощью камеры,
    приземление с уточнением и возврат на базу.
    """

    def __init__(self,
                 drone: Pion,
                 base_coords: np.ndarray,
                 delivery_points: List[Tuple[float, float, float, float]],
                 camera: Optional[BaseCamera] = None,
                 mission_keys: Optional[List[str]] = None,
                 show: bool = False) -> None:
        """
        :param drone: Объект дрона для доставки.
        :param base_coords: Координаты базы (точка возврата).
        :param delivery_points: Список точек для кандидатов (x, y, z, yaw).
        :param camera: Экземпляр камеры для видеопотока. Если не передан, будет создан RTSPCamera по умолчанию.
        :param mission_keys: Список QR-кодов, для которых необходимо выполнить доставку.
        :param show: Флаг отображения видеопотока.
        """
        self.drone: Pion = drone
        self.base_coords: np.ndarray = base_coords
        self.show = show
        self.delivery_points: List[Tuple[float, float, float, float]] = delivery_points
        self.mission_keys: List[str] = mission_keys if mission_keys is not None else []
        # Если камера не передана, создаём камеру по умолчанию по RTSP
        if camera is None:
            rtsp_url = f'rtsp://{self.drone.ip}:8554/front'
            self.camera = RTSPCamera(rtsp_url)
        else:
            self.camera = camera

    def initialize_drone(self) -> None:
        """
        Инициализирует дрона-доставщика: арминг, взлёт и установка скорости.
        """
        time.sleep(1)
        self.smart_take_off()
        time.sleep(7)

    def smart_take_off(self) -> None:
        """
        Функция для принудительного взлёта.
        """
        print("Smart take off is beginning")
        while self.drone.xyz[2] < 0.3:
            time.sleep(1)
            self.drone.arm()
            time.sleep(0.5)
            self.drone.takeoff()
        time.sleep(7)
        print("Smart take off is ending")

    def deliver_to_target(self, target_key: str,
                          target_coord: np.ndarray,
                          scaling_factor: float = 0.5,
                          threshold: float = 0.05,
                          dict_of_points_to_return: Optional[Dict[str, np.ndarray]] = None
                          ) -> np.ndarray:
        """
        Выполняет доставку для одного QR-кода. В качестве основной candidate точки используется
        найденная координата (target_coord) из сканирования. Если корректировка не удается,
        дополнительно перебираются точки из delivery_points. После корректировки дрон возвращается на базу.

        :param target_key: Имя QR-кода (цель).
        :type target_key: str
        :param target_coord: Исходная (усреднённая) координата цели, полученная сканером.
        :type target_coord: np.ndarray
        :param scaling_factor: Коэффициент масштабирования скорости корректировки.
        :type scaling_factor: float
        :param threshold: Порог точности (в метрах) для завершения корректировки.
        :type threshold: float
        :param dict_of_points_to_return: Опциональный словарь с точками возврата для цели.
        :type dict_of_points_to_return: Optional[Dict[str, np.ndarray]]
        :return: Финальная координата дрона после корректировки.
        :rtype: np.ndarray
        """
        final_coord: Optional[np.ndarray] = None
        candidate_points: List[Tuple[float, float, float, float]] = [(target_coord[0], target_coord[1], 1.7, 0)]
        candidate_points.extend(self.delivery_points)
        for point in candidate_points:
            if self.drone.xyz[2] < 0.3:
                print("Взлет")
                self.smart_take_off()
            print(f"Попытка доставки для '{target_key}' с candidate точкой {point}")
            self.drone.speed_flag = True
            self.drone.goto_from_outside(*point)
            self.drone.speed_flag = False
            time.sleep(2)
            # Используем универсальную камеру для получения кадра
            finished_targets: List[str] = []
            frame = self.camera.get_cv_frame()
            if frame is not None:
                height, width, _ = frame.shape
                frame_center = (width // 2, height // 2)
            else:
                frame_center = (0, 0)
            finished_targets, final_coord = move_to_target(self.drone, self.camera, finished_targets,
                                                           show=self.show, key=target_key,
                                                           scaling_factor=scaling_factor,
                                                           threshold=threshold, time_break=15)
            print(f"Для '{target_key}' получены координаты: {final_coord}")
            self.drone.land()
            time.sleep(15)
            get_box(drone=self.drone)
            time.sleep(15)
            self.smart_take_off()
            if final_coord is not None:
                break
        if final_coord is None:
            print(f"Не удалось уточнить позицию для '{target_key}'")
            final_coord = target_coord
        if dict_of_points_to_return:
            if target_key in dict_of_points_to_return:
                self.return_to_point(dict_of_points_to_return[target_key])
        else:
            self.return_to_base()
        return final_coord

    def deliver_all(self,
                    targets: Dict[str, np.ndarray],
                    coordinates_of_bases: Dict[str, np.ndarray]
                    ) -> Dict[str, np.ndarray]:
        """
        Для каждого QR-кода из словаря целей выполняется доставка:
        дрон уточняет позицию, приземляется с корректировкой и возвращается на базу.

        :param targets: Словарь QR-кодов с исходными координатами, полученными сканером.
        :type targets: Dict[str, np.ndarray]
        :param coordinates_of_bases: Словарь с точками возврата для каждой цели.
        :type coordinates_of_bases: Dict[str, np.ndarray]
        :return: Словарь доставленных координат для каждого QR-кода.
        :rtype: Dict[str, np.ndarray]
        """
        delivered: Dict[str, np.ndarray] = {}
        for key, coord in targets.items():
            print(f"Запуск доставки для цели '{key}' с координатами {coord}")
            final_coord = self.deliver_to_target(key, coord, dict_of_points_to_return=coordinates_of_bases)
            delivered[key] = final_coord
        print("Доставка завершена. Доставленные координаты:", delivered)
        return delivered

    def return_to_base(self) -> None:
        """
        Возвращает дрона-доставщика на базу.

        :return: None
        """
        print(f"\n\nReturn to base: {self.base_coords}\n\n")
        self.smart_take_off()
        self.drone.goto_from_outside(*self.base_coords)
        self.drone.speed_flag = False
        self.drone.land()
        time.sleep(20)

    def return_to_point(self, point: np.ndarray) -> None:
        """
        Возвращает дрона-доставщика на заданную точку.

        :param point: Таргетная точка возврата (x, y, z, yaw).
        :type point: np.ndarray
        :return: None
        """
        print(f"\n\nReturn to point {point}\n\n")
        self.drone.goto_from_outside(*point)
        self.drone.speed_flag = False
        self.drone.land()
        time.sleep(20)
        drop_box(self.drone)


class MissionController:
    def __init__(self, scanner: DroneScanner, deliverer: DroneDeliverer,
                 coordinates_of_bases: dict, targets: list):
        """
        :param scanner: Экземпляр DroneScanner для сканирования QR-кодов.
        :param deliverer: Экземпляр DroneDeliverer для выполнения доставки.
        :param coordinates_of_bases: Словарь точек возврата для каждой цели.
        :param targets: Список имен QR-кодов (целей), которые необходимо обработать.
        """
        self.scanner = scanner
        self.deliverer = deliverer
        self.coordinates_of_bases = coordinates_of_bases
        self.targets = targets
        self.emergency_event = threading.Event()  # Флаг экстренной ситуации
        self.weather_thread = threading.Thread(target=self.weather_monitor, daemon=True)

    def weather_monitor(self):
        """
        Имитирует изменение погодных условий:
          - 0–2 минуты: зеленая погода (все в норме);
          - 2–3 минуты: желтая погода (предупреждение: через минуту ураган);
          - 3–4 минуты: красная погода (экстренная ситуация – флаг устанавливается).
        После 4-й минуты, если флаг был установлен, он сбрасывается (погода нормализуется).
        """
        mission_start = time.time()
        while True:
            elapsed = time.time() - mission_start
            if elapsed < 120 / 2:
                print("[Weather Monitor] Погода: зеленая – всё в норме.")
            elif elapsed < 180 / 2:
                print("[Weather Monitor] Погода: желтая – через минуту будет ураган! Инициирую экстренный возврат.")
                if not self.emergency_event.is_set():
                    self.emergency_event.set()
            elif elapsed < 240 / 2:
                print("[Weather Monitor] Погода: красная – ураган! Сидим!.")
            elif elapsed < 300 / 2:
                print("[Weather Monitor] Погода: красная – ураган! Сидим!.")
                if self.emergency_event.is_set():
                    self.emergency_event.clear()
                    print("[Weather Monitor] Погода нормализовалась. Экстренный режим завершён.")
            else:
                if self.emergency_event.is_set():
                    self.emergency_event.clear()
                    print("[Weather Monitor] Погода нормализовалась. Экстренный режим завершён.")
                break
            time.sleep(10)

    def run_mission(self):
        """
        Основной метод выполнения миссии:
          1. Запускается мониторинг погодных условий.
          2. Выполняется сканирование QR-кодов.
          3. Отбираются целевые коды.
          4. Запускается доставка для найденных целей. В каждом этапе доставки проверяется,
             установлен ли флаг экстренной ситуации. Если да, выполняется процедура приостановки:
             дрон возвращается на базу, ждет нормализации и затем возобновляет миссию с сохраненной точки.
          5. По завершении миссии дрон возвращается на базу.
        """
        print("run_mission")

        # Запуск мониторинга погоды в отдельном потоке
        ## self.weather_thread.start()

        # Выполнение сканирования
        self.scanner.initialize_drone()  # это мы просто взлетаем
        scanned_results = self.scanner.execute_scan(self.emergency_event)
        print("Результаты сканирования:", scanned_results)

        # Запус РТС
        ip = "127.0.0.1"
        targets = {  # Если оставить пустым, то бот сразу отправится домой
            "targ1": [scanned_results["Wood 1"][0], scanned_results["Wood 1"][1]],  # Для левой стороны
            "targ2": [scanned_results["Stone 1"][0], scanned_results["Stone 1"][1]],  # Для левой стороны
        }
        process1_bot = multiprocessing.Process(target=bot_process, daemon=True,
                                               args=[ip, name_l, targets, lines, obs_med, obs_large, home_point_l,
                                                     sklads_l, vzaimosv])
        ##process1_bot.start()

        ip = "10.1.100.127"
        targets = {  # Если оставить пустым, то бот сразу отправится домой
            "targ1": [scanned_results["Wood 2"][0], scanned_results["Wood 2"][1]],  # Для левой стороны
            "targ2": [scanned_results["Stone 2"][0], scanned_results["Stone 2"][1]],  # Для левой стороны
        }
        process2_bot = multiprocessing.Process(target=bot_process, daemon=True,
                                               args=[ip, name_r, targets, lines, obs_med, obs_large, home_point_r,
                                                     sklads_r, vzaimosv])
        ##process2_bot.start()

        self.scanner.return_to_base()
        # Отбор только тех целей, которые указаны в self.targets
        matched_targets = {k: v for k, v in scanned_results.items() if k in self.targets}
        if not matched_targets:
            print("Целевые QR-коды не обнаружены. Завершаем миссию.")
            return

        # Передаём флаг экстренной ситуации в объект доставщика (если требуется)
        self.deliverer.emergency_event = self.emergency_event
        self.deliverer.initialize_drone()
        # Выполнение доставки с учетом возможного экстренного возврата
        delivered_results = self.deliverer.deliver_all(matched_targets, self.coordinates_of_bases)
        print("Итоговые доставленные координаты:", delivered_results)

        # Завершение миссии – возврат на базу
        self.deliverer.return_to_base()
