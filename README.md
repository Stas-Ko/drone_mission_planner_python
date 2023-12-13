# drone_mission_planner_python

ЗАДАЧА:

Создать скрипт, который выполняет взлет и направляет коптер из точки А в точку Б в режиме AltHold, после прибытия поворачивается на азимут (yaw) 350 градусов


Точка А = Точка Влета = Точка Домой : [ 50.450739, 30.461242 ]
Точка Б = [ 50.443326, 30.448078 ] Высота : 100 м



Описание
Создан скрипт на Python 3.6 с использованием библиотек dronekit и pymavlink для автоматизированного взлета коптера и направления его из точки А в точку Б в режиме AltHold. 

Инструкции по использованию:

Запустите Mission Planner.
Перейдите во вкладку PLAN.
В правой части экрана заполните широту и долготу для Точки А (Точка Влета) - [50.450739, 30.461242].
Перейдите во вкладку SIMULATION.
В нижней части экрана выберите Multirotor.
Запустите скрипт на Python 3.6 с установленными библиотеками dronekit и pymavlink.

Запуск скрипта:

python drone_launch.py

Скрипт автоматически выполнит взлет и направление коптера из точки А в точку Б в режиме AltHold. По достижении цели, коптер повернется на азимут (yaw) 350 градусов.



Важно:
Убедитесь, что у вас установлен Python 3.6.
Установите библиотеки dronekit и pymavlink перед запуском скрипта.
Програма Mission Planner

pip install dronekit pymavlink
