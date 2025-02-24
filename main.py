from typing import List, Tuple, Dict, Optional
from pion import Pion
import numpy as np
from rzd.drone_controller import *
import sys

if __name__ == "__main__":
    # Параметры для дрона-сканера
    scout_drone_1 = Pion(
        ip="127.0.0.1",
        mavlink_port=8001,
        logger=False,
        dt=0.0,
        accuracy=0.08,
        count_of_checking_points=5
    )
    # Координаты базы
    base_coords_scanner = np.array([-3.92 - 2.14, 0, 0])
    # По каким координатам мы пройдемся для сканирования
    # пролетим вдоль жд
    h = 2
    offset = 1.4
    # зона сканирования - 2х2 м
    x = -5
    y = -5 + offset
    scan_points_list_1 = []
    # проход туда
    while x < 6 or y < 6:
        scan_points_list_1.append([x, y, h, 135])
        x += offset * 2
        y += offset * 2
        pass
    # проход обратно
    x,y,zz,aa = scan_points_list_1[ len(scan_points_list_1)-1 ] # смещаемся от последней точки
    x += offset
    y -= offset
    while x > -6 or y > -6:
        scan_points_list_1.append([x, y, h, 315])
        x -= offset * 2
        y -= offset * 2
        pass

    scan_points = np.array(scan_points_list_1)

    # Выбираем режим камеры: для симулятора можно использовать SocketCamera,
    # для реального дрона – RTSPCamera. Здесь пример с RTSP.
    scanner_camera = RTSPCamera(rtsp_url=f'rtsp://{scout_drone_1.ip}:8554/front')
    scanner_1 = DroneScanner(drone=scout_drone_1, base_coords=base_coords_scanner,
                             scan_points=scan_points, camera=scanner_camera, show=True)

    # Параметры для дрона-доставщика
    delivery_drone = Pion(
        ip="10.1.100.211",
        mavlink_port=5656,
        logger=False,
        dt=0.0,
        accuracy=0.08,
        count_of_checking_points=5
    )
    base_coords_deliverer = np.array([3.5, 3.3, 1.5, 0])

    delivery_points: List[Tuple[float, float, float, float]] = [
        (0, 0, 0, 0),
        (0, 0, 0, 0),
        (0, 0, 0, 0)
    ]
    # Для доставщика также можно передать камеру; если не передана, будет создан RTSPCamera по умолчанию.
    deliverer_camera = RTSPCamera(rtsp_url=f'rtsp://{delivery_drone.ip}:8554/front')
    deliverer = DroneDeliverer(drone=delivery_drone, base_coords=base_coords_deliverer,
                               delivery_points=delivery_points, camera=deliverer_camera,
                               mission_keys=["Box 2 2", "Box 2 1"], show=True)

    # Координаты точек возврата для каждой цели
    coordinates_of_bases = {
        "Box 2 2": np.array([-3.2, -0.6, 1.5, 0]),
        "Box 2 1": np.array([0.8, 3, 1.5, 0])
    }
    # Список целевых QR-кодов
    targets = ["Box 2 2", "Box 2 1"]

    # Создание MissionController и запуск миссии
    mission_controller = MissionController(scanner, deliverer, coordinates_of_bases, targets)
    mission_controller.run_mission()
