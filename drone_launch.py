from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

# Подключение к симулятору Mission Planner через MAVProxy
connection_string = "tcp:127.0.0.1:5762"

#Рассчитывает расстояние в метрах между двумя глобальными координатами.
def get_distance_metres(location1, location2):
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

try:
    # Подключение к дрону
    print("Connecting to the vehicle...")
    vehicle = connect(connection_string, wait_ready=True, timeout=60)
    print("Connected to the vehicle!")

    # Выводим информацию о дроне
    print(f"Vehicle: {vehicle}")
    print(f"Armed: {vehicle.armed}")
    print(f"Mode: {vehicle.mode.name}")
    print(f"Location: {vehicle.location.global_frame}")
    print(f"Heading: {vehicle.heading}")

    # Взлет на высоту 10 метров
    target_altitude = 100.0
    print(f"Arming and taking off to {target_altitude} meters...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    vehicle.simple_takeoff(target_altitude)

    # Ждем, пока дрон достигнет целевой высоты
    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude!")
            break
        time.sleep(1)

    # Ожидаем несколько секунд перед началом движения
    time.sleep(5)

    # Перемещение к точке Б = Точка Куда лететь
    target_location_destination = LocationGlobalRelative(50.443326, 30.448078, target_altitude)
    print(f"Moving to destination point B: {target_location_destination.lat}, {target_location_destination.lon}, {target_location_destination.alt}...")

    # Ждем, пока дрон достигнет целевой точки
    vehicle.simple_goto(target_location_destination)

    while get_distance_metres(vehicle.location.global_frame, target_location_destination) > 1.0:
        remaining_distance = get_distance_metres(vehicle.location.global_frame, target_location_destination)
        print(f"Distance to destination point B: {remaining_distance} meters")
        time.sleep(1)

    print("Reached destination point B!")

    # Поворот на азимут (yaw) 350 градусов
    target_yaw = 350
    print(f"Turning to yaw {target_yaw} degrees...")

    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        target_yaw,  # param 1 (yaw angle in degrees)
        0,  # param 2 (yaw speed in degrees per second)
        1,  # param 3 (direction: 1 - clockwise, -1 - counterclockwise)
        0,  # param 4 (relative offset in degrees)
        0, 0, 0)  # param 5-7 (not used)

    vehicle.send_mavlink(msg)
    time.sleep(5)  # Подождем несколько секунд, чтобы дрон завершил поворот

    # Вырубаем двигатели и заканчиваем программу
    print("Disarming and closing connection...")
    vehicle.armed = False
    time.sleep(1)
    vehicle.close()

except Exception as e:
    print(f"Error connecting to or controlling the vehicle: {e}")
