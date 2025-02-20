from typing import List, Tuple, Dict, Optional
from pion import Pion
import numpy as np
from rzd.drone_controller import *
import sys

if __name__ == "__main__":
    # Параметры для дрона-сканера
    scout_drone = Pion(
        ip="10.1.100.217",
        mavlink_port=5656,
        logger=False,
        dt=0.0,
        accuracy=0.08,
        count_of_checking_points=5
    )
    # Координаты базы
    base_coords_scanner = np.array([3.4, 2.4, 1.0, 0])
    # По каким координатам мы пройдемся для сканирования
    scan_points = np.array([
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]
    ])
    # Выбираем режим камеры: для симулятора можно использовать SocketCamera,
    # для реального дрона – RTSPCamera. Здесь пример с RTSP.
    scanner_camera = RTSPCamera(rtsp_url=f'rtsp://{scout_drone.ip}:8554/front')
    scanner = DroneScanner(drone=scout_drone, base_coords=base_coords_scanner,
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