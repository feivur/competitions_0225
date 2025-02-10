from typing import List, Tuple, Dict, Optional
from pion import Pion
import numpy as np
from rzd import *
import sys


def main() -> None:
    """
    Основная функция. Сначала создается объект DroneScanner для сканирования QR-кодов.
    После сканирования результат (словарь QR-кодов с координатами) передается объекту DroneDeliverer,
    который выполняет доставку для каждой найденной цели.
    
    :return: None
    """
    # Параметры для дрона-сканера
    scout_drone = Pion(
        ip="10.1.100.215",
        mavlink_port=5656,
        logger=False,
        dt=0.0,
        accuracy=0.08, # Точность позиционирования при goto_from_outside()
        count_of_checking_points=5 # Количество точек, которым проверяется отклонение accuracy
    )
    base_coords = np.array([0, 0, 0, 0])
    scan_points = np.array([
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]
    ])
    
    # Список целевых QR-кодов, которые необходимо найти
    targets: List[str] = ["Box 2 2", "Box 2 1"]
    # Точки возврата для каждой цели (например, где должен приземлиться дрон после доставки)
    coordinates_of_bases = {targets[0]: np.array([0, 0, 0, 0]), targets[1]: np.array([0, 0, 0, 0])}

    # Создаем сканирующий дрон и выполняем сканирование
    scanner = DroneScanner(drone=scout_drone, base_coords=base_coords, scan_points=scan_points, show=True)
    scanned_results = scanner.execute_scan()
    print("Результаты сканирования (усредненные координаты):", scanned_results)
    
    # Отбираем только те QR-коды, которые входят в список targets
    matched_targets = {k: v for k, v in scanned_results.items() if k in targets}
    if not matched_targets:
        print("Целевые QR-коды не обнаружены. Завершаем работу.")
        return
    
    # Параметры для дрона-доставщика
    delivery_drone = Pion(
        ip="10.1.100.211",
        mavlink_port=5656,
        logger=False,
        dt=0.0,
        accuracy=0.08,
        count_of_checking_points=5
    )
    delivery_points: List[Tuple[float, float, float, float]] = [
        (0, 0, 0, 0),
        (0, 0, 0, 0),
        (0, 0, 0, 0)
    ]
    
    # Создаем дрона-доставщика и выполняем доставку для найденных QR-кодов
    deliverer = DroneDeliverer(drone=delivery_drone, 
                                base_coords=np.array([0, 0, 0, 0]),
                                delivery_points=delivery_points,
                                mission_keys=targets,
                                show=True)
    delivered_results = deliverer.deliver_all(matched_targets, coordinates_of_bases=coordinates_of_bases)
    print("Итоговые доставленные координаты:", delivered_results)
    deliverer.return_to_base()

if __name__ == "__main__":
    main()

