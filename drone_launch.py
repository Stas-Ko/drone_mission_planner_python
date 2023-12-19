from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from math import radians, sin, cos, sqrt, atan2


# Флаг для управления поворотом дрона во время полета
turn_away = True

# Флаг для управления остановкой движения вперед-назад при приближении к цели
stop_fly = True

# Флаг для управления поворотом дрона на 350 градусов после остановки
turn_away350 = True


def calculate_distance(lat1, lon1, lat2, lon2):
    """
    Вычисляет расстояние между двумя глобальными координатами в метрах.

    Parameters:
    - lat1, lon1: Широта и долгота первой точки (в градусах).
    - lat2, lon2: Широта и долгота второй точки (в градусах).

    Returns:
    - Расстояние между точками в метрах.
    """
    # Радиус Земли в метрах
    R = 6371000.0

    # Преобразование координат в радианы
    lat1_rad = radians(lat1)
    lon1_rad = radians(lon1)
    lat2_rad = radians(lat2)
    lon2_rad = radians(lon2)

    # Разница координат
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    # Формула гаверсинусов для расчета расстояния
    a = sin(dlat / 2)**2 + cos(lat1_rad) * cos(lat2_rad) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    # Расстояние в метрах
    distance = R * c

    return distance


# Подключение к дрону (замените 'tcp:127.0.0.1:5762' на ваш адрес)
connection_string = "tcp:127.0.0.1:5762"
print("Connecting to the vehicle...")
vehicle = connect(connection_string, wait_ready=True, timeout=60)
print("Connected to the vehicle!")

# Выводим информацию о дроне
print(f"Vehicle: {vehicle}")
print(f"Armed: {vehicle.armed}")
print(f"Mode: {vehicle.mode.name}")
print(f"Location: {vehicle.location.global_frame}")
print(f"Heading: {vehicle.heading}")

# Задаем координаты точки, к которой дрон должен лететь
target_location = (50.443326, 30.448078)

try:
    # Устанавливаем режим ALT_HOLD
    vehicle.mode = VehicleMode("ALT_HOLD")


    # Проверяем, что дрон включен и в режиме ALT_HOLD
    vehicle.armed = True
    time.sleep(1)
    print(f"Mode: {vehicle.mode.name}")

    # Устанавливаем высоту в 100 метров
    target_altitude = 100
    vehicle.simple_takeoff(target_altitude)

    print(f"Altitude set to {target_altitude} meters. Gaining altitude...")

    # Управляем высотой
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt

        # Задаем значение для увеличения высоты
        if current_altitude < target_altitude:
            vehicle.channels.overrides['3'] = 2000  # Замените на нужное значение
            # Если дрон находится ближе к целевой высоте, уменьшаем мощность для точного набора высоты
            if current_altitude >= target_altitude - 3:
                vehicle.channels.overrides['3'] = 1425  # Замените на нужное значение

                # Первый разворот для коррекции направления при наборе высоты
                if turn_away:
                    vehicle.channels.overrides['4'] = 1376  # Замените на нужное значение
                    time.sleep(3)
                    vehicle.channels.overrides = {}
                    turn_away = False  # После выполнения разворота прекращаем повторные развороты

                # Дополнительные коррекции, если дрон находится ближе к целевой высоте
                if current_altitude >= target_altitude - 0.1:
                    vehicle.channels.overrides['3'] = 1420  # Замените на нужное значение


        else:
            # Задаем значение для уменьшения высоты, если выше целевой
            vehicle.channels.overrides['3'] = 1400

            # Получаем текущие глобальные координаты дрона
            current_location = (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)

            # Вычисляем расстояние между текущим положением и целевыми координатами
            distance_to_target = calculate_distance(current_location[0], current_location[1], target_location[0],
                                                    target_location[1])

            # При удалении от цели больше 25 метров снижаем скорость движения вперед
            if int(distance_to_target) > 25 and stop_fly:
                vehicle.channels.overrides['2'] = 1000
            else:
                vehicle.channels.overrides['2'] = 1500
                stop_fly = False  # После снижения скорости прекращаем повторное снижение


                # При первом вхождении в блок разворота на 350 градусов
                if turn_away350:
                    print("Reached the target coordinates.")
                    time.sleep(5)
                    print("Initiating 350-degree turn...")
                    vehicle.channels.overrides['4'] = 1616
                    time.sleep(5)
                    vehicle.channels.overrides = {}
                    print("350-degree turn completed.")
                    turn_away350 = False  # После выполнения разворота прекращаем повторные развороты

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Interrupted by the user.")

finally:
    # Выключаем двигатели и закрываем соединение при завершении
    vehicle.channels.overrides['3'] = 1500  # Установите значение для выключения двигателей
    vehicle.armed = False
    time.sleep(1)
    vehicle.close()
