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
angles1 = deque([0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)
angles2 = deque([0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)
control_values = deque([0] * BUFFER_SIZE, maxlen=BUFFER_SIZE)

# Флаг для остановки потока
running = True

def read_serial_data(ser):
    """Поток для чтения данных с последовательного порта"""
    global angles1, angles2, control_values, running
    while running:
        if ser.in_waiting >= PACKET_SIZE:
            byte = ser.read(1)
            if byte == HEADER[0:1]:
                if ser.read(1) == HEADER[1:2]:
                    data = ser.read(PACKET_SIZE - 2)
                    if len(data) == PACKET_SIZE - 2 and data[-2:] == TERMINATOR:
                        # Распаковка данных
                        angles = struct.unpack('<2f', data[0:8])
                        control = struct.unpack('<I', data[8:12])[0]  # Control (uint32)
                        
                        angles1.append(angles[0])
                        angles2.append(angles[1])
                        control_values.append(control)

# Настройка графического интерфейса
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, size=(2000, 1200))
win.setWindowTitle("Real-Time Visualization")

# График углов (angles1 и angles2)
plot_angles = win.addPlot(title="Angles (Angle 1 and Angle 2)")
curve_angle1 = plot_angles.plot(pen='b', name="Angle 1")
curve_angle2 = plot_angles.plot(pen='r', name="Angle 2")

# Установка диапазона оси Y для углов
plot_angles.setYRange(-10, 190)
plot_angles.setLabel('left', 'Angles (degrees)')
plot_angles.setLabel('bottom', 'Time (frames)')

# Легенда для углов
legend_angles = pg.LegendItem(offset=(-100, 1))
legend_angles.setParentItem(plot_angles)
legend_angles.addItem(curve_angle1, "Target Angle")
legend_angles.addItem(curve_angle2, "Current Angle")

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
    global angles1, angles2, control_values
    curve_angle1.setData(list(range(len(angles1))), list(angles1))
    curve_angle2.setData(list(range(len(angles2))), list(angles2))
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
