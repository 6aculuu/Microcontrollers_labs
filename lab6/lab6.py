import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import serial
import struct
import threading
from collections import deque

# Настройки последовательного порта
SERIAL_PORT = 'COM23'    # Укажите ваш порт
BAUDRATE = 900000        # Скорость передачи
PACKET_SIZE = 16         # Размер пакета (SimulinkPacket: 2 + 8 + 4 + 2)
HEADER = b'\xAA\xBB'     # Заголовок
TERMINATOR = b'\xCC\xDD' # Терминатор

# Размер буфера данных
BUFFER_SIZE = 10000

# Очереди для данных
speed1 = deque([0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)
speed2 = deque([0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)
control_values = deque([0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)

# Флаг для остановки потока
running = True

def read_serial_data(ser):
    """Поток для чтения данных с последовательного порта"""
    global speed1, speed2, control_values, running
    while running:
        if ser.in_waiting >= PACKET_SIZE:
            byte = ser.read(1)
            if byte == HEADER[0:1]:
                if ser.read(1) == HEADER[1:2]:
                    data = ser.read(PACKET_SIZE - 2)
                    if len(data) == PACKET_SIZE - 2 and data[-2:] == TERMINATOR:
                        # Распаковка данных
                        speeds = struct.unpack('<2f', data[0:8])
                        control = struct.unpack('<I', data[8:12])[0]  # Control (uint32)

                        speed1.append(speeds[0] * 60000 / 817)
                        speed2.append(speeds[1]* 60000 / 817)
                        control_values.append(control)

# Настройка графического интерфейса
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, size=(2000, 1200))
win.setWindowTitle("Real-Time Speed Visualization")

# График скоростей (speed1 и speed2)
plot_speeds = win.addPlot(title="Speeds (Speed 1 and Speed 2)")
curve_speed1 = plot_speeds.plot(pen='b', name="Speed 1")
curve_speed2 = plot_speeds.plot(pen='r', name="Speed 2")

# Добавление оси Y = 0
zero_line = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('w', width=1, style=pg.QtCore.Qt.DashLine))
plot_speeds.addItem(zero_line)
zero_line.setZValue(-100)

# Установка диапазона оси Y для скоростей
plot_speeds.setYRange(-235, 235)  # Скорости могут быть как положительными, так и отрицательными
plot_speeds.setLabel('left', 'Speed (units)')
plot_speeds.setLabel('bottom', 'Time (frames)')

# Легенда для скоростей
legend_speeds = pg.LegendItem(offset=(-100, 1))
legend_speeds.setParentItem(plot_speeds)
legend_speeds.addItem(curve_speed1, "Target Speed")
legend_speeds.addItem(curve_speed2, "Current Speed")

# Добавляем второй график для управления (control)
win.nextRow()  # Размещаем новый график ниже
plot_control = win.addPlot(title="Control Signal")
curve_control = plot_control.plot(pen='g', name="Control")

# Установка диапазона оси Y для управления
plot_control.setYRange(0, 100)
plot_control.setLabel('left', 'Control Signal (%)')
plot_control.setLabel('bottom', 'Time (frames)')

def update_graph():
    """Функция для обновления графиков"""
    global speed1, speed2, control_values
    curve_speed1.setData(list(range(len(speed1))), list(speed1))
    curve_speed2.setData(list(range(len(speed2))), list(speed2))
    curve_control.setData(list(range(len(control_values))), list(control_values))

# Настройка последовательного порта
ser = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=0.01)

# Запуск потока для чтения данных
serial_thread = threading.Thread(target=read_serial_data, args=(ser,), daemon=True)
serial_thread.start()

# Таймер для обновления графиков
timer = QtCore.QTimer()
timer.timeout.connect(update_graph)
timer.start(10)  # Обновление каждые 10 мс

# Запуск приложения
if __name__ == '__main__':
    try:
        print("Чтение данных и отображение графиков...")
        QtWidgets.QApplication.instance().exec_()
    except KeyboardInterrupt:
        print("Программа завершена.")
    finally:
        running = False
        serial_thread.join()
        ser.close()
