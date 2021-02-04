import sys, time
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PIL import ImageQt, Image
import json
from UI.ui import Ui_Form as MainWindow
from UI.items import Ui_Form as Items
from UI.action_item import Ui_Form as ActionItemsUi
import serial
import math
import re
import os


class SteeringItemClipBoard(object):
    instance = None
    init_flag = None

    def __init__(self):
        """此处设计为，如果已经存在实例时，不再执行初始化"""
        if self.init_flag:
            return
        else:
            self.init_flag = True
        self.data = None

    def __new__(cls, *args, **kwargs):
        """如果当前没有实例时，调用父类__new__方法，生成示例，有则返回保存的内存地址。"""
        if not cls.instance:
            cls.instance = super().__new__(cls)
        return cls.instance


class ClipBoard(object):
    instance = None
    init_flag = None

    def __init__(self):
        """此处设计为，如果已经存在实例时，不再执行初始化"""
        if self.init_flag:
            return
        else:
            self.init_flag = True
        self.data = None

    def __new__(cls, *args, **kwargs):
        """如果当前没有实例时，调用父类__new__方法，生成示例，有则返回保存的内存地址。"""
        if not cls.instance:
            cls.instance = super().__new__(cls)
        return cls.instance


class SerialServo(object):
    # 串口类
    instance = None
    init_flag = None

    def __init__(self):
        """此处设计为，如果已经存在实例时，不再执行初始化"""
        if self.init_flag:
            return
        else:
            self.init_flag = True

        self.serialHandle = None
        # 命令库
        self.command = {"SERVO_MOVE_TIME_WRITE": 1,  # 发送旋转指令
                        'SERVO_MOVE_TIME_READ': 2,
                        'SERVO_MOVE_TIME_WAIT_WRITE': 7,
                        'SERVO_MOVE_TIME_WAIT_READ': 8,
                        'SERVO_MOVE_START': 11,
                        'SERVO_MOVE_STOP': 12,  # 舵机急停
                        'SERVO_ID_WRITE': 13,  # 该指令会重新给舵机写入 ID 值
                        'SERVO_ID_READ': 14,
                        'SERVO_ANGLE_OFFSET_ADJUST': 17,  # 该指令到达舵机，舵机将立即转动进行偏差调整。通过此指令调整好的偏差值不支持掉电保存
                        'SERVO_ANGLE_OFFSET_WRITE': 18,  # 保存偏差值，并支持掉电保存
                        'SERVO_ANGLE_OFFSET_READ': 19,  # 读取舵机已设定偏差值
                        'SERVO_ANGLE_LIMIT_WRITE': 20,  # 限制舵机最大最小角度
                        'SERVO_ANGLE_LIMIT_READ': 21,  # 读取舵机的已设定的限制值
                        'SERVO_VIN_LIMIT_WRITE': 22,  # 设定舵机允许电压范围
                        'SERVO_VIN_LIMIT_READ': 23,  # 读取舵机已设定电压范围
                        'SERVO_TEMP_MAX_LIMIT_WRITE': 24,  # 舵机工作温度范围设定
                        'SERVO_TEMP_MAX_LIMIT_READ': 25,  # 读取已设定工作温度范围
                        'SERVO_TEMP_READ': 26,  # !!读取舵机实时温度
                        'SERVO_VIN_READ': 27,  # !!读取舵机实时电压
                        "SERVO_POS_READ": 28,  # !!读取舵机当前实际角度
                        'SERVO_OR_MOTOR_MODE_WRITE': 29,  # 设定舵机工作模式，舵机模式/减速机模式
                        'SERVO_OR_MOTOR_MODE_READ': 30,  # 读取舵机相关参数
                        'SERVO_LOAD_OR_UNLOAD_WRITE': 31,  # 设定舵机是否掉电
                        'SERVO_LOAD_OR_UNLOAD_READ': 32,  # 读取舵机内部当前状态
                        'SERVO_LED_CTRL_WRITE': 33,
                        'SERVO_LED_CTRL_READ': 34,
                        'SERVO_LED_ERROR_WRITE': 35,
                        'SERVO_LED_ERROR_READ': 36,  # 读取舵机警告值，可获知是否堵转
                        }

    def servo_write_cmd(self, steering_id, cmd, par1=None, par2=None):
        if self.serialHandle is None:
            return
        # 清空接收缓存
        self.serialHandle.flushInput()
        # 如果发送转动命令，校验角度范围是否在0到1000以内。
        if cmd == 1:
            # 保护舵机，防止过度转动
            assert 10 <= par1 <= 990
            assert 50 <= par2 <= 5000
        # print(cmd, par1, par2)
        # return False
        buf = bytearray(b'\x55\x55')
        try:
            _len = 3  # 若命令是没有参数的话数据长度就是3
            buf1 = bytearray(b'')

            # 对参数进行处理
            if par1 is not None:
                _len += 2  # 数据长度加2
                buf1.extend([(0xff & par1), (0xff & (par1 >> 8))])  # 分低8位 高8位 放入缓存
            if par2 is not None:
                _len += 2
                buf1.extend([(0xff & par2), (0xff & (par2 >> 8))])  # 分低8位 高8位 放入缓存
            buf.extend([(0xff & steering_id), (0xff & _len), (0xff & cmd)])
            buf.extend(buf1)  # 追加参数

            # 计算校验和
            _sum = 0x00
            for b in buf:  # 求和
                _sum += b
            _sum = _sum - 0x55 - 0x55  # 去掉命令开头的两个 0x55
            _sum = ~_sum  # 取反
            buf.append(0xff & _sum)  # 取低8位追加进缓存
            self.serialHandle.write(buf)  # 发送
        except Exception as _e:
            print(_e)
            return False
        return buf

    def servo_read_cmd_by_other(self, steering_id, cmd):
        if self.serialHandle is None:
            return
        # 发送读取位置命令
        self.servo_write_cmd(steering_id, cmd)
        # 稍作延时，等待接收完毕
        time.sleep(0.007)
        # 获取接收缓存中的字节数
        count = self.serialHandle.inWaiting()
        date = None
        # 如果接收到的数据不空
        if count != 0:
            if cmd == 28:
                recv_data = self.serialHandle.read(count)
                if count == 8:
                    # 第一第二个字节等于0x55, 第5个字节是0x1C 就是 28 就是 位置读取命令的命令号
                    if recv_data[0] == 0x55 and recv_data[1] == 0x55:
                        # 将接收到的字节数据拼接成完整的位置数据
                        date = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                return date  # 返回读取到的位置
            if cmd == 25:
                print('count长度：', count)
                recv_data = self.serialHandle.read(count)
                print('recv_data:', recv_data)
                if count == 7:
                    # 第一第二个字节等于0x55, 第5个字节是0x1C 就是 28 就是 位置读取命令的命令号 and recv_data[1] == 0x55
                    if recv_data[0] == 0x55:
                        # 将接收到的字节数据拼接成完整的位置数据
                        date = 0xffff & (recv_data[2] | (0xff00 & (recv_data[3] << 8)))
                return date
            if cmd == 27:
                print('count长度：', count)
                recv_data = self.serialHandle.read(count)
                print('recv_data:', recv_data)
                if count == 7:
                    # 第一第二个字节等于0x55, 第5个字节是0x1C 就是 28 就是 位置读取命令的命令号 and recv_data[1] == 0x55
                    if recv_data[0] == 0x55:
                        # 将接收到的字节数据拼接成完整的位置数据
                        date = 0xffff & (recv_data[2] | (0xff00 & (recv_data[3] << 8)))
                return date

    def servo_read_cmd(self, steering_id, cmd):
        if self.serialHandle is None:
            return
        # 发送读取位置命令
        send_buf = self.servo_write_cmd(steering_id, cmd)
        if send_buf is False:
            return

        # 稍作延时，等待接收完毕
        time.sleep(0.007)
        # 获取接收缓存中的字节数
        len_send_buf = len(send_buf)
        # count = self.serialHandle.inWaiting() - len_send_buf
        count = self.serialHandle.inWaiting()
        date = None
        # 如果接收到的数据不空
        print('send_buf', send_buf, '---', count)
        if count > 0:
            recv_data = self.serialHandle.read(count)  # 读取接收到的数据
            print('recv_data', recv_data)
            recv_data = recv_data[len_send_buf:]
            print('recv_data', recv_data)
            count = count - len_send_buf
            print('cmd', cmd, count)
            if cmd == 28:
                if count == 8:
                    # 第一第二个字节等于0x55, 第5个字节是0x1C 就是 28 就是 位置读取命令的命令号
                    print('recv_data_3', recv_data)
                    if recv_data[0] == 0x55 and recv_data[1] == 0x55:
                        # 将接收到的字节数据拼接成完整的位置数据
                        date = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                return date  # 返回读取到的位置
            if cmd == 25:
                if count == 7:
                    # 第一第二个字节等于0x55, 第5个字节是0x1C 就是 28 就是 位置读取命令的命令号 and recv_data[1] == 0x55
                    if recv_data[0] == 0x55:
                        # 将接收到的字节数据拼接成完整的位置数据
                        date = 0xffff & (recv_data[2] | (0xff00 & (recv_data[3] << 8)))
                return date
            if cmd == 27:
                if count == 7:
                    # 第一第二个字节等于0x55, 第5个字节是0x1C 就是 28 就是 位置读取命令的命令号 and recv_data[1] == 0x55
                    if recv_data[0] == 0x55:
                        # 将接收到的字节数据拼接成完整的位置数据
                        date = 0xffff & (recv_data[2] | (0xff00 & (recv_data[3] << 8)))
                return date
            if cmd == 19:
                print('recv_data-7', recv_data)
                if recv_data[0] == 0x55:
                    # 将接收到的字节数据拼接成完整的位置数据
                    date = 0xffff & recv_data[5]
                    print('data', date)
                return date

    def close(self):
        self.serialHandle.close()

    @classmethod
    def get_ports(cls):
        import serial.tools.list_ports
        plist = list(serial.tools.list_ports.comports())
        port_list = []
        if len(plist) <= 0:
            print("没有发现端口!")
        else:
            for i in plist:
                print("可用端口名>>>", i)
                port_list.append(i)
        return port_list

    def __new__(cls, *args, **kwargs):
        """如果当前没有实例时，调用父类__new__方法，生成示例，有则返回保存的内存地址。"""
        if not cls.instance:
            cls.instance = super().__new__(cls)
        return cls.instance


class Steering(object):
    # 舵机类，相同的参数只允许创建一个对象
    steering_id = {}

    def __init__(self, s_id: int):
        if Steering.steering_id[s_id]:
            return
        else:
            Steering.steering_id[s_id] = self

        self.serial_servo = SerialServo()
        self.s_id = s_id
        self.min_degree = 10
        self.max_degree = 990
        self.last_degree = 10
        self.load = 0
        self.last_max_d = 0
        self.deviation_angle = None
        self.command = {"SERVO_MOVE_TIME_WRITE": 1,  # 发送旋转指令
                        'SERVO_MOVE_TIME_READ': 2,
                        'SERVO_MOVE_TIME_WAIT_WRITE': 7,
                        'SERVO_MOVE_TIME_WAIT_READ': 8,
                        'SERVO_MOVE_START': 11,
                        'SERVO_MOVE_STOP': 12,  # 舵机急停
                        'SERVO_ID_WRITE': 13,  # 该指令会重新给舵机写入 ID 值
                        'SERVO_ID_READ': 14,
                        'SERVO_ANGLE_OFFSET_ADJUST': 17,  # 该指令到达舵机，舵机将立即转动进行偏差调整。通过此指令调整好的偏差值不支持掉电保存
                        'SERVO_ANGLE_OFFSET_WRITE': 18,  # 保存偏差值，并支持掉电保存
                        'SERVO_ANGLE_OFFSET_READ': 19,  # 读取舵机已设定偏差值
                        'SERVO_ANGLE_LIMIT_WRITE': 20,  # 限制舵机最大最小角度
                        'SERVO_ANGLE_LIMIT_READ': 21,  # 读取舵机的已设定的限制值
                        'SERVO_VIN_LIMIT_WRITE': 22,  # 设定舵机允许电压范围
                        'SERVO_VIN_LIMIT_READ': 23,  # 读取舵机已设定电压范围
                        'SERVO_TEMP_MAX_LIMIT_WRITE': 24,  # 舵机工作温度范围设定
                        'SERVO_TEMP_MAX_LIMIT_READ': 25,  # 读取已设定工作温度范围
                        'SERVO_TEMP_READ': 26,  # !!读取舵机实时温度
                        'SERVO_VIN_READ': 27,  # !!读取舵机实时电压
                        "SERVO_POS_READ": 28,  # !!读取舵机当前实际角度
                        'SERVO_OR_MOTOR_MODE_WRITE': 29,  # 设定舵机工作模式，舵机模式/减速机模式
                        'SERVO_OR_MOTOR_MODE_READ': 30,  # 读取舵机相关参数
                        'SERVO_LOAD_OR_UNLOAD_WRITE': 31,  # 设定舵机是否掉电
                        'SERVO_LOAD_OR_UNLOAD_READ': 32,  # 读取舵机内部当前状态
                        'SERVO_LED_CTRL_WRITE': 33,
                        'SERVO_LED_CTRL_READ': 34,
                        'SERVO_LED_ERROR_WRITE': 35,
                        'SERVO_LED_ERROR_READ': 36,  # 读取舵机警告值，可获知是否堵转
                        }

    def change_id(self, to_id: int):
        """修改舵机ID"""
        self.serial_servo.servo_write_cmd(self.s_id, self.command['SERVO_ID_WRITE'], to_id)

    def save_deviation_degree(self):
        """
        保存偏移量
        :return:
        """
        self.serial_servo.servo_write_cmd(self.s_id, self.command['SERVO_ANGLE_OFFSET_WRITE'])

    def set_deviation_degree(self, value):
        self.onload()
        if value < 0:
            value = 256 + value
        if 0 <= value <= 255:
            self.serial_servo.servo_write_cmd(self.s_id, self.command['SERVO_ANGLE_OFFSET_ADJUST'], value)

    def get_deviation_degree(self):
        try:
            deviation_degree = self.serial_servo.servo_read_cmd(self.s_id, self.command['SERVO_ANGLE_OFFSET_READ'])
            if deviation_degree > 127:
                deviation_degree = deviation_degree - 256
            return deviation_degree
        except:
            return False

    def rotation_angle(self, radian, run_time=None, speed=None):
        if run_time is None and speed is None:
            raise BaseException('参数不完整')

        # 角度转换 范围 0~1000，对应舵机角度的 0~240
        degree = int(math.degrees(radian) / 0.24)
        # print(self.s_id, degree)

        if abs(self.last_degree - degree) < 2:
            # if self.s_id in _s_list:
            # print('id:{},degree_value（已转换）：{}，防止因为精度问题来回摆动~,上次值：{}'.format(self.s_id, degree, self.last_degree))
            return

        # 旋转到绝对值角度。角度不允许超限
        if self.min_degree > degree:
            degree = self.min_degree

        if self.max_degree < degree:
            degree = self.max_degree

        if run_time:
            run_time = int(run_time)
        else:
            run_time = int(abs(self.last_degree - degree) / speed)
            if run_time == 0:
                run_time = 1000

        if not 0 < run_time < 9000:
            raise BaseException('时间参数有误')

        if not self.load:
            self.onload()

        self.serial_servo.servo_write_cmd(self.s_id, 1, degree, run_time)
        # 更新最新角度
        self.last_degree = degree

    def rotation_degree(self, degree_int: int, run_time=None, speed=None):
        if run_time is None and speed is None:
            raise BaseException('参数不完整')

        # 角度转换 范围 0~1000，对应舵机角度的 0~240
        degree = degree_int
        # print(self.s_id, degree)

        # if abs(self.last_degree - degree) < 2:
        #     # if self.s_id in _s_list:
        #     # print('id:{},degree_value（已转换）：{}，防止因为精度问题来回摆动~,上次值：{}'.format(self.s_id, degree, self.last_degree))
        #     return

        # 旋转到绝对值角度。角度不允许超限
        if self.min_degree > degree:
            degree = self.min_degree

        if self.max_degree < degree:
            degree = self.max_degree

        if run_time:
            run_time = int(run_time)
        else:
            run_time = int(abs(self.last_degree - degree) / speed)
            if run_time == 0:
                run_time = 1000

        if not 0 < run_time < 9000:
            raise BaseException('时间参数有误')

        if not self.load:
            self.onload()

        self.serial_servo.servo_write_cmd(self.s_id, 1, degree, run_time)
        # d = self.last_degree - degree
        # if self.laxt_max_d < d:
        #     self.laxt_max_d = d
        #     print('self.laxt_max_d', self.laxt_max_d)
        # 更新最新角度
        # self.last_degree = degree

    def get_radian(self):
        """
        :return: 返回弧度  范围 0~1000，对应舵机角度的 0~240
        """
        # 获取当前角度
        try:
            degree = self.serial_servo.servo_read_cmd(self.s_id, 28)
        except:
            degree = None

        if degree is not None:
            self.last_degree = degree
            return math.radians(degree * 0.24)
        else:
            return False

    def get_degree(self):
        """
        :return: 返回舵机值  范围 0~1000，对应舵机角度的 0~240
        """
        try:
            degree = self.serial_servo.servo_read_cmd(self.s_id, 28)
            return degree
        except BaseException as e:
            print(e)

            return False

    def stop(self):
        self.serial_servo.servo_write_cmd(self.s_id, 12)

    def get_setting_temperature(self):
        """获取设置的温度"""
        temperature = self.serial_servo.servo_read_cmd(self.s_id, 25)
        return temperature

    def set_temp(self):
        """设置该设备的温度上限"""
        self.serial_servo.servo_write_cmd(self.s_id, 24, 85)
        return True

    def get_voltage(self):
        """获取当前舵机的电压"""
        voltage = self.serial_servo.servo_read_cmd(self.s_id, 27)
        return voltage

    def unload(self):
        # 设备卸力
        print(self.s_id, '设备准备卸力')
        if self.load:
            self.load = 0
            self.serial_servo.servo_write_cmd(self.s_id, 31, 0)
            print(self.s_id, '设备已卸力')

    def onload(self):
        # 设备上电
        if not self.load:
            self.load = 1
            self.serial_servo.servo_write_cmd(self.s_id, 31, 1)

    def __new__(cls, *args, **kwargs):
        _id = kwargs['s_id']
        if _id not in cls.steering_id:
            cls.steering_id[_id] = None
            return super().__new__(cls)
        return cls.steering_id[_id]


class HorizontalSlider(QSlider):
    def mousePressEvent(self, QMouseEvent):
        super().mousePressEvent(QMouseEvent)
        self.parent().item_click_sign.emit(self.parent())


class QtItems(QWidget, Items):
    item_click_sign = pyqtSignal(QWidget)
    item_move_end_sign = pyqtSignal()
    item_info_sign = pyqtSignal(str)
    paste_sign = pyqtSignal(QWidget)
    copy_sign = pyqtSignal(QWidget)

    def __init__(self, s_id):
        super().__init__()
        self.setupUi(self)
        self.widget.setStyleSheet('background:#555555')
        self.resize(160, 95)
        self.setLayout(self.horizontalLayout)
        self.s_id = s_id
        self.label.setText('ID:{}'.format(self.s_id))

        self.c = None

        self.is_active = True
        self.change_active(True)

        self.is_press = False
        self.is_selected = False
        self.is_moved = False
        self.is_move_enable = False

        self.steering_engine = Steering(s_id=self.s_id)
        self.is_get_deviation_value = False

        # self.setContextMenuPolicy(Qt.CustomContextMenu)
        # self.customContextMenuRequested.connect(self.show_menu)

        self.set_ui()

    def text_line_clicked_event(self, m_event):
        if m_event.button() == Qt.LeftButton:
            pass
        else:
            self.show_menu(m_event.pos())
            return
        pass

    def show_menu(self, point):
        menu = QMenu(self)

        action_1 = QAction("复制", menu)
        action_1.triggered.connect(self.copy)

        action_2 = QAction("粘贴", menu)
        action_2.triggered.connect(self.paste)

        action_3 = QAction("编辑ID", menu)
        action_3.triggered.connect(self.change_s_id)

        action_4 = QAction("删除", menu)
        action_4.triggered.connect(self.delete)

        menu.addAction(action_1)
        menu.addAction(action_2)
        menu.addAction(action_3)
        menu.addAction(action_4)

        dest_point = self.mapToGlobal(point)
        menu.exec_(dest_point)

    def change_s_id(self, ):
        self.c = ChangeSID()
        self.c.change_s_id_sign.connect(self.c_s_id)
        self.c.show()

    def c_s_id(self, value: int):
        if not value:
            return
        items = self.parent().get_all_child_item()
        s_id_list = [item.s_id for item in items]
        print('s_id_list', s_id_list)
        if value not in s_id_list:
            self.s_id = value
            self.label.setText('ID:{}'.format(self.s_id))

            self.c.close()
        else:
            self.show_info('该ID已存在不允许修改')

    def delete(self):
        flag = QMessageBox.question(self, '警告', '是否删除舵机',
                                    QMessageBox.Yes | QMessageBox.No,
                                    QMessageBox.Yes)
        if flag == QMessageBox.Yes:
            self.deleteLater()

    def copy(self):
        self.copy_sign.emit(self)
        # SteeringItemClipBoard().data = int(self.lineEdit.text())

    def paste(self):
        self.paste_sign.emit(self)
        # s_data = SteeringItemClipBoard().data
        # if s_data:
        #     self.lineEdit.setText(str(s_data))

    def change_active(self, flag=None):

        if flag is None:
            if self.is_active:
                self.is_active = False
            else:
                self.is_active = True
        elif flag is True:
            self.is_active = True
        else:
            self.is_active = False

        if self.is_active:
            self.label.setStyleSheet('background:green')
        else:
            self.label.setStyleSheet('background:red')

    def show_info(self, text: str):
        self.item_info_sign.emit(text)

    def save_deviation_value(self):
        if self.is_get_deviation_value:
            value = self.spinBox.value()
            self.show_info('保存偏差：{}'.format(value))
            self.steering_engine.save_deviation_degree()
        else:
            self.show_info('s_id:{},请先获取舵机偏差'.format(self.s_id))

    def horizontal_slider_mouse_press_event(self, event):
        self.item_click_sign.emit(self)
        self.horizontalSlider.mousePressEvent(event)
        # super().horizontal_slider_mouse_press_event(event)

    def set_ui(self):
        self.label.mouseDoubleClickEvent = self.lable_mouse_double_click_event
        self.horizontalSlider = HorizontalSlider(self)
        self.horizontalSlider.setGeometry(QRect(10, 40, 141, 16))
        self.horizontalSlider.setOrientation(Qt.Horizontal)

        self.horizontalSlider.setMinimum(0)
        self.horizontalSlider.setMaximum(1000)
        self.horizontalSlider.setValue(500)
        self.horizontalSlider.valueChanged.connect(self.horizontal_slider_change)

        # self.horizontalSlider.mousePressEvent = self.horizontal_slider_mouse_press_event

        self.lineEdit.setText('500')
        self.lineEdit.setToolTip('舵机目标角度')
        self.lineEdit.focusInEvent = self.click
        self.lineEdit.editingFinished.connect(self.degree_change)
        self.lineEdit.mousePressEvent = self.text_line_clicked_event

        self.lineEdit_2.setText('300')
        self.lineEdit_2.setToolTip('舵机运动时间')
        self.lineEdit_2.focusInEvent = self.click
        self.lineEdit_2.textChanged.connect(self.time_line_edit_change)

        self.spinBox.setMinimum(-150)
        self.spinBox.setMaximum(150)
        self.spinBox.setValue(0)
        self.spinBox.setToolTip('舵机偏差，注：获取偏差再设置')
        self.spinBox.valueChanged.connect(self.deviation_value_change)

    def lable_mouse_double_click_event(self, e):
        self.change_active()

    def get_run_time(self):
        return int(self.lineEdit_2.text())

    def set_run_time(self, value):
        self.lineEdit_2.setText(str(value))

    def get_target_degree(self):
        return int(self.lineEdit.text())

    def set_target_degree(self, value):
        self.lineEdit.setText(str(value))
        self.horizontalSlider.setValue(value)

    def time_line_edit_change(self, text):
        t = re.findall('^[0-9]*', text)
        if t:
            self.lineEdit_2.setText(t[0])

        value = self.lineEdit_2.text()
        if value:
            value = int(value)
            if value > 9000:
                self.lineEdit_2.setText(str(9000))
            else:
                self.lineEdit_2.setText(str(value))

    def run_steering(self, *args):
        print('rorate_steering', int(self.lineEdit.text()))
        degree = int(self.lineEdit.text())
        run_time = int(self.lineEdit_2.text())
        self.steering_engine.rotation_degree(degree, run_time=run_time)

    def get_degree(self):
        degree = self.steering_engine.get_degree()
        if degree is False:
            self.show_info('舵机：{}，读取角度失败'.format(self.s_id))
            return
        if degree is None:
            self.show_info('舵机：{}，读取角度失败'.format(self.s_id))
            return
        self.lineEdit.setText(str(degree))
        self.horizontalSlider.setValue(degree)

    def click(self, *args):
        self.item_click_sign.emit(self)

    def deviation_value_change(self, *args):
        if not self.is_get_deviation_value:
            self.show_info('请先获取偏移角度')
            self.spinBox.setValue(0)
            return
        self.steering_engine.set_deviation_degree(self.spinBox.value())
        print('deviation_value_change', self.spinBox.value())

    def get_deviation_value(self):
        deviation_value = self.steering_engine.get_deviation_degree()
        print('deviation_value', deviation_value)
        if deviation_value is False:
            return
        if deviation_value is None:
            return

        self.is_get_deviation_value = True
        self.spinBox.setValue(deviation_value)

    def get_deviation_current_value(self):
        return self.spinBox.value()

    def set_deviation_current_value(self, value: int):
        self.spinBox.setValue(value)

    def degree_change(self, *args):
        try:
            value = int(self.lineEdit.text())
        except:
            return
        self.horizontalSlider.setValue(value)

    def horizontal_slider_change(self, *args):
        value = args[0]
        # self.item_click_sign.emit(self)

        self.lineEdit.setText(str(value))
        self.run_steering(value)

    def mousePressEvent(self, m_event):
        if m_event.button() == Qt.LeftButton:
            self.is_press = True
            self.w_last_pos = m_event.windowPos()
            self.last_pos = self.pos()
            self.item_click_sign.emit(self)
        else:
            self.show_menu(m_event.pos())

    def mouseMoveEvent(self, m_event):
        if self.is_press and self.is_move_enable:
            self.is_moved = True
            _pos = m_event.windowPos()
            d_pos = self.w_last_pos - _pos
            # print(d_pos)
            pos = self.last_pos - d_pos
            self.move(pos.x(), pos.y())

    def mouseReleaseEvent(self, m_event):
        self.is_press = False
        if self.is_moved:
            # 保存相对位置
            self.item_move_end_sign.emit()

        self.is_moved = False

    def select(self, flag):
        """
        :param flag: 是否指定状态，不指定时，每次选择进行切换
        :return:
        """
        if flag:
            self.is_selected = True
            self.widget.setStyleSheet('background:#57D0EF')
        else:
            self.is_selected = False
            self.widget.setStyleSheet('background:#555555')
        # print(self.is_selected, self.s_id)

    def unload(self):
        self.steering_engine.unload()


class ActionItem(QWidget, ActionItemsUi):
    edit_action_sign = pyqtSignal(QWidget)
    add_action_item_sign_upper = pyqtSignal(QWidget)
    add_action_item_sign_below = pyqtSignal(QWidget)
    delete_action_item_sign = pyqtSignal()
    update_action_item_sign = pyqtSignal()

    def __init__(self, name=None):
        super().__init__()
        self.setupUi(self)
        self.setLayout(self.horizontalLayout)
        self.is_selected = False

        self.name = name
        self.data = None

        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.show_menu)
        self.set_ui()

    def set_ui(self):
        # self.lable_1 = QLabel(self)
        # self.lable_1.move(0, 10)
        # self.lable_1.resize(200, 10)

        # self.pushButton = QPushButton(self)
        # self.pushButton.resize(50, 20)
        # self.pushButton.move(self.width() - self.pushButton.width() - 50,
        #                      (self.height() - self.pushButton.height()) / 2)
        # self.pushButton.move(50, 20)
        # self.pushButton.setText('编辑')

        self.label.setText('')

        self.pushButton.clicked.connect(self.edit_action)

    def edit_action(self):
        if self.is_selected:
            flag = QMessageBox.question(self, '警告', '是否更新当前动作',
                                        QMessageBox.Yes | QMessageBox.No,
                                        QMessageBox.Yes)
            if flag == QMessageBox.Yes:
                self.update_action_item_sign.emit()

        self.select(True)
        self.edit_action_sign.emit(self)

    def mouseDoubleClickEvent(self, QMouseEvent):
        self.select()

    def select(self, flag=None):
        """
        :param flag: 是否指定状态，不指定时，每次选择进行切换
        :return:
        """
        if flag is None:
            if self.is_selected:
                self.is_selected = False
            else:
                self.is_selected = True
            return
        if flag:
            self.is_selected = True
        else:
            self.is_selected = False

        if self.is_selected:
            self.pushButton.setText('更新')
            self.pushButton.setStyleSheet('background:#57D0EF')
        else:
            self.pushButton.setText('编辑')
            self.pushButton.setStyleSheet('background:#E1E1E1')
        # print('setStyleSheet')

    def update_data(self, data):
        if not data:
            self.label_2.setText('请完善信息~')
        else:
            self.data = data
            self.name = self.data['name']
            if not self.name:
                self.name = '未命名'
            self.label_2.setText('{},time:{}'.format(self.name, self.data['sleep_time']))
            print(self.data)

            text = ''
            for steering_data in self.data['steerings_data']:
                text += 'ID {} # {} ,'.format(steering_data['s_id'], steering_data['degree'])

            if len(text) > 100:
                text = text[:100] + '...'
            self.label_3.setText(text)

    def show_menu(self, point):
        menu = QMenu(self)
        action_1 = QAction("编辑", menu)
        action_1.triggered.connect(self.select)

        action_2 = QAction("删除", menu)
        action_2.triggered.connect(self.delete_item)

        action_3 = QAction("复制", menu)
        action_3.triggered.connect(self.copy_data)

        action_4 = QAction("粘贴", menu)
        action_4.triggered.connect(self.paste_data)

        action_5 = QAction("在上方插入一行", menu)
        action_5.triggered.connect(lambda: self.add_action_item_sign_upper.emit(self))

        action_6 = QAction("在下方插入一行", menu)
        action_6.triggered.connect(lambda: self.add_action_item_sign_below.emit(self))

        menu.addAction(action_1)
        menu.addAction(action_2)
        menu.addAction(action_3)
        menu.addAction(action_4)
        menu.addAction(action_5)
        menu.addAction(action_6)

        dest_point = self.mapToGlobal(point)
        menu.exec_(dest_point)

    def delete_item(self):
        list_w = self.parent().parent()
        list_w.takeItem(list_w.currentRow())
        self.delete_action_item_sign.emit()

    def copy_data(self):
        clipboard = ClipBoard()
        clipboard.data = self.data
        # clipboard = QApplication.clipboard()
        # clipboard.setText(self.lable_1.text())

    def paste_data(self):
        clipboard = ClipBoard()
        if clipboard.data:
            self.update_data(clipboard.data)
            if self.is_selected:
                self.edit_action_sign.emit(self)


class StreeingShowWidget(QWidget):
    show_info_sign = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        # self.setStyleSheet('background:red')
        # self.paintEvent(None)

        self.NewRect = None
        self.is_shift_pressed = False
        self.rb = QRubberBand(QRubberBand.Rectangle, self)

        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.show_menu)

        self.init_steerings()

    def init_steerings(self):
        try:
            with open('item_pos.json', 'r') as f:
                item_pos = json.loads(f.read())
        except:
            item_pos = []

        for item_data in item_pos:

            item = self.add_steering(item_data['s_id'])
            item.move(item_data['pos_x'], item_data['pos_y'])
            try:
                item.set_deviation_current_value(item_data['deviation_value'])
            except:
                pass

    def show_info(self, text):
        self.show_info_sign.emit(text)

    def paintEvent(self, evt):
        opt = QStyleOption()
        opt.initFrom(self)
        painter = QPainter(self)
        # 反锯齿
        painter.setRenderHint(QPainter.Antialiasing)

    def add_steering(self, s_id=1):
        item = QtItems(s_id)
        item.move(0, 0)

        if self.parent.is_move_enable:
            item.is_move_enable = True

        item.setParent(self)
        item.item_click_sign.connect(self.click_item)
        item.item_move_end_sign.connect(self.save_item_pos)
        item.item_info_sign.connect(self.show_info)
        item.copy_sign.connect(self.steering_item_copy)
        item.paste_sign.connect(self.steering_item_paste)
        item.show()

        return item

    def show_menu(self, point):
        menu = QMenu(self)
        action_1 = QAction("添加舵机", menu)
        action_1.triggered.connect(self.add_steering)

        menu.addAction(action_1)

        dest_point = self.mapToGlobal(point)
        menu.exec_(dest_point)

    def mouseMoveEvent(self, evt):
        super().mouseMoveEvent(evt)
        self.NewPos = evt.pos()
        NewRect = QRect(self.originPos, self.NewPos)
        self.rb.setGeometry(NewRect.normalized())
        self.NewRect = NewRect.normalized()

    def mousePressEvent(self, evt):
        if evt.button() == Qt.RightButton:
            return

        super().mousePressEvent(evt)
        if not self.is_shift_pressed:
            for item in self.findChildren(QtItems):
                item.select(False)

        self.originPos = evt.pos()
        rect = QRect(self.originPos, QSize())
        self.rb.setGeometry(rect)
        self.rb.show()

    def mouseReleaseEvent(self, evt):
        if not self.NewRect:
            return
            # 这里面主要判断复选框控件的区域 cb.geometry() 是否包含在橡皮筋矩形self.NewRect里面
        for item in self.findChildren(QtItems):
            if not self.is_shift_pressed:
                item.select(False)
            if self.NewRect.intersected(item.geometry()):
                item.select(True)
        self.NewRect = None
        self.rb.hide()

    def keyReleaseEvent(self, e):
        if e.key() == Qt.Key_Shift or e.key() == Qt.Key_Control:
            self.is_shift_pressed = False

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Shift or event.key() == Qt.Key_Control:
            self.is_shift_pressed = True
        if (event.key() == Qt.Key_C) and QApplication.keyboardModifiers() == Qt.ControlModifier:
            print("ctrl + C")
            self.steering_item_copy()
        if (event.key() == Qt.Key_V) and QApplication.keyboardModifiers() == Qt.ControlModifier:
            print("ctrl + V")
            self.steering_item_paste()
        if (event.key() == Qt.Key_Q) and QApplication.keyboardModifiers() == Qt.ControlModifier:
            print("ctrl + Q")
            for item in self.widget.findChildren(QtItems):
                item.select(True)

    def click_item(self, _item):
        if not self.is_shift_pressed:
            for item in self.findChildren(QtItems):
                item.select(False)
        if _item.is_selected:
            _item.select(False)
        else:
            _item.select(True)

    def save_item_pos(self):
        print('save_item_pos')
        data_list = []
        for item in self.findChildren(QtItems):
            data_dict = {'s_id': item.s_id,
                         'pos_x': item.x(),
                         'pos_y': item.y(),
                         'deviation_value': item.get_deviation_current_value()
                         }
            data_list.append(data_dict)

        with open('item_pos.json', 'w') as f:
            f.write(json.dumps(data_list))

    def steering_item_copy(self, widget=None):
        # items = self.get_selected_item()
        # if len(items) > 1:
        #     self.show_info('只能复制1个')
        #     return
        _ = {}
        if widget:
            _['run_time'] = widget.get_run_time()
            _['degree'] = widget.get_target_degree()
            _['is_active'] = widget.is_active
        else:
            items = self.get_selected_item()
            if len(items) == 1:
                widget = items[0]
                _['run_time'] = widget.get_run_time()
                _['degree'] = widget.get_target_degree()
                _['is_active'] = widget.is_active
            else:
                self.show_info('只能选择一个舵机ITEM进行赋值')
                return
        SteeringItemClipBoard().data = _
        self.show_info('已复制ID：{}的数据'.format(widget.s_id))

    def steering_item_paste(self, widget=None):
        steerings_data_dict = SteeringItemClipBoard().data
        if not steerings_data_dict:
            return
        items = self.get_selected_item()
        if widget:
            items.append(widget)
        # if not items:
        #     items = [widget]
        # print(items)
        for steering_item in items:
            steering_item.set_run_time(steerings_data_dict['run_time'])
            steering_item.set_target_degree(steerings_data_dict['degree'])
            steering_item.change_active(steerings_data_dict['is_active'])

    def get_selected_item(self, flag=False):
        _ = []
        for item in self.findChildren(QtItems):
            if flag:
                _.append(item)
            else:
                if item.is_selected:
                    _.append(item)
        return _

    def get_all_child_item(self):
        _ = []
        for item in self.findChildren(QtItems):
            _.append(item)
        return _


class ChangeSID(QWidget):
    change_s_id_sign = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.lin_edit_1 = QLineEdit(self)
        self.lin_edit_1.editingFinished.connect(self.finish_edit)

    def finish_edit(self, *args):
        try:
            value = int(self.lin_edit_1.text())
        except:
            return
        self.change_s_id_sign.emit(value)


class QtMain(QWidget, MainWindow):

    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.setWindowTitle('串口舵机动作组编辑器')

        self.current_action_group_file_path = None

        self.run_action_group_thread = RunActionGroup()
        self.is_run_action_group = False
        self.run_action_group_thread.end_run_sign.connect(self.run_action_group_is_end)

        self.is_serial_opened = False
        self.serial_servo = SerialServo()
        self.is_move_enable = False

        self.setLayout(self.horizontalLayout)
        self.set_ui()
        self.change_item_move_enable()

        self.actions_group_list = []
        self.current_action_index = None
        self.add_action(0)

    def set_ui(self):

        self.verticalLayout.removeWidget(self.widget)
        self.widget.deleteLater()

        self.widget = StreeingShowWidget(self)
        self.widget.show_info_sign.connect(self.show_info)
        self.verticalLayout.insertWidget(1, self.widget)

        self.verticalLayout.setStretch(0, 2)
        self.verticalLayout.setStretch(1, 50)
        self.verticalLayout.setStretch(2, 15)

        self.lineEdit.setText('500')
        self.lineEdit_3.setText('500')

        self.pushButton.clicked.connect(self.save_steering_deviation)
        self.pushButton_2.clicked.connect(self.get_steering_deviation)
        self.pushButton_3.clicked.connect(self.get_steering_degree)
        self.pushButton_4.clicked.connect(self.batch_steering_data)
        self.pushButton_5.clicked.connect(self.deviation_to_zero)

        self.pushButton_6.clicked.connect(self.unload_all_steerings)
        self.pushButton_7.clicked.connect(self.deal_serial)
        self.pushButton_8.clicked.connect(self.change_item_move_enable)

        self.pushButton_9.clicked.connect(self.update_action)

        self.pushButton_11.clicked.connect(self.open_action_file)
        self.pushButton_12.clicked.connect(self.save_action_to_file)
        self.pushButton_13.clicked.connect(lambda: self.save_action_to_file(True))

        self.pushButton_14.clicked.connect(lambda: self.set_steerings_active(True))
        self.pushButton_15.clicked.connect(lambda: self.set_steerings_active(False))

        self.pushButton_16.clicked.connect(self.run_action_group)
        self.checkBox.stateChanged.connect(self.checkBox_stateChanged)

    def checkBox_stateChanged(self, *args):
        print(args)
        self.run_action_group_thread.is_loop = self.get_run_action_group_thread_is_loop()

    def run_action_group_is_end(self):
        self.is_run_action_group = False

        self.pushButton_16.setText('运行动作组')
        self.pushButton_16.setStyleSheet('background:#314446;color:white')

    def run_action_group(self):
        if self.is_run_action_group:
            self.run_action_group_thread.is_active = False
            self.pushButton_16.setText('正在停止中~')
            self.pushButton_16.setStyleSheet('background:yellow')
        else:
            data_list = []
            for i in range(self.listWidget.count()):
                item = self.listWidget.itemWidget(self.listWidget.item(i))
                if item.data:
                    data_list.append(item.data)
            if not data_list:
                self.show_info('没有有效动作')
                return

            self.is_run_action_group = True
            self.run_action_group_thread.action_group_data = data_list

            self.run_action_group_thread.is_active = True
            self.run_action_group_thread.is_loop = self.get_run_action_group_thread_is_loop()
            self.run_action_group_thread.start()
            self.pushButton_16.setText('点击停止运行')
            self.pushButton_16.setStyleSheet('background:red;color:white')
        pass

    def get_run_action_group_thread_is_loop(self):
        print(self.checkBox.isChecked())
        return self.checkBox.isChecked()

    def open_action_file(self):
        abs_path = os.path.abspath(__file__)
        top_dir = os.path.dirname(abs_path)

        filename = QFileDialog.getOpenFileName(self, 'open file', top_dir)
        if not filename[0]:
            return
        else:
            self.current_action_group_file_path = filename[0]

        flag = QMessageBox.question(self, '警告', '打开并覆盖当前动作组',
                                    QMessageBox.Yes | QMessageBox.No,
                                    QMessageBox.Yes)
        if flag == QMessageBox.No:
            return

        with open(filename[0], 'r') as f:
            data_list = f.read()
        try:
            data_list = json.loads(data_list)
        except:
            data_list = None

        for i in range(self.listWidget.count()):
            # item = self.listWidget.itemWidget(self.listWidget.item(i))
            self.listWidget.takeItem(0)

        if not data_list:
            self.add_action(0)
            return

        for index, action_data in enumerate(data_list):
            self.add_action(index, action_data)

    def set_steerings_active(self, flag=False):
        items = self.get_selected_item()
        if not items:
            self.show_info('请先选择舵机')
            return
        for item in items:
            item.change_active(flag)

    def save_action_to_file(self, is_save_as_flag=False):
        abs_path = os.path.abspath(__file__)
        top_dir = os.path.dirname(abs_path)

        if self.current_action_group_file_path is None or is_save_as_flag:
            filename = QFileDialog.getSaveFileName(self, 'save_file.json', top_dir)
            print(filename)
            if not filename[0]:
                return
            else:
                self.current_action_group_file_path = filename[0]

        with open(self.current_action_group_file_path, 'w') as f:
            data_list = []
            for i in range(self.listWidget.count()):
                item = self.listWidget.itemWidget(self.listWidget.item(i))
                if item.data:
                    data_list.append(item.data)
            if data_list:
                data = json.dumps(data_list)
                f.write(data)

        self.show_info('文件已保存')

    def add_action_item_upper_or_below(self, widget, flag=None):
        if flag is None:
            self.add_action()
            return

        current_row = None

        for row in range(self.listWidget.count()):
            item = self.listWidget.itemWidget(self.listWidget.item(row))
            if item == widget:
                current_row = row
                break
        if current_row is None:
            return
        if flag == 'upper':
            self.add_action(current_row)
        else:
            self.add_action(current_row + 1)

    def delete_action_item(self):
        if not self.listWidget.count():
            self.add_action()

    def add_action(self, row=None, action_data=None):
        item = QListWidgetItem()
        item.setSizeHint(QSize(200, 50))
        action_item = ActionItem()
        action_item.update_data(action_data)

        action_item.edit_action_sign.connect(self.edit_action)
        action_item.add_action_item_sign_upper.connect(
            lambda widget: self.add_action_item_upper_or_below(widget, 'upper'))
        action_item.add_action_item_sign_below.connect(
            lambda widget: self.add_action_item_upper_or_below(widget, 'below'))
        action_item.delete_action_item_sign.connect(self.delete_action_item)
        action_item.update_action_item_sign.connect(self.update_action)

        if row is None:
            self.listWidget.addItem(item)
            # self.listWidget.insertItem(self.listWidget.count()+1, item)
        else:
            self.listWidget.insertItem(row, item)
        self.listWidget.setItemWidget(item, action_item)

    def edit_action(self, widget):
        # 编辑动作组
        for i in range(self.listWidget.count()):
            item = self.listWidget.itemWidget(self.listWidget.item(i))
            if item != widget:
                item.select(False)

        # 填写数据
        data = widget.data
        if not data:
            return
        self.lineEdit_3.setText(str(data['sleep_time']))
        self.lineEdit_4.setText(str(data['name']))
        for steering_item in self.widget.findChildren(QtItems):
            steering_item.change_active(False)
            for steerings_data_dict in data['steerings_data']:
                if steering_item.s_id == steerings_data_dict['s_id']:
                    steering_item.change_active(True)
                    steering_item.set_run_time(steerings_data_dict['run_time'])
                    steering_item.set_target_degree(steerings_data_dict['degree'])
                    break

    def update_action(self):
        action_item = None
        for i in range(self.listWidget.count()):
            item = self.listWidget.itemWidget(self.listWidget.item(i))
            if item.is_selected:
                action_item = item

        if action_item is None:
            self.show_info('请在左侧列表右键，选择或新增动作')
            return
        # print(action_item.data)
        data = {}
        data['steerings_data'] = []
        for item in self.widget.findChildren(QtItems):
            if item.is_active:
                _ = {}
                _['s_id'] = item.s_id
                _['run_time'] = item.get_run_time()
                _['degree'] = item.get_target_degree()
                data['steerings_data'].append(_)
        if data['steerings_data']:
            data['sleep_time'] = self.get_action_sleep_time()
            data['name'] = self.lineEdit_4.text()
            if data['sleep_time']:
                action_item.update_data(data)

    def get_action_sleep_time(self):
        text = self.lineEdit_3.text()
        if text:
            try:
                value = int(text)
                return value
            except:
                return None
        return None

    def show_info(self, text: str):
        self.label.setText(text)
        print(text)

    def deal_serial(self):
        if not self.is_serial_opened:
            try:
                adr = self.lineEdit_2.text()
                serial_handle = serial.Serial(adr, 115200)

                self.serial_servo.serialHandle = serial_handle
                self.show_info('serialHandle 串口已打开')
                self.pushButton_7.setText('关闭串口')
                self.is_serial_opened = True
            except:
                self.show_info('serialHandle 串口打开失败')
        else:
            if self.serial_servo.serialHandle:
                self.serial_servo.close()
                self.pushButton_7.setText('打开串口')
                self.show_info('serialHandle 串口已关闭')
                self.is_serial_opened = False

    def change_item_move_enable(self):
        items = self.get_selected_item(flag=True)
        if self.is_move_enable:
            for item in items:
                item.is_move_enable = False
            self.is_move_enable = False
            self.pushButton_8.setText('设置舵机ITEM可拖动')
            self.show_info('设置不可拖动成功')
        else:
            for item in items:
                item.is_move_enable = True
            self.is_move_enable = True
            self.pushButton_8.setText('设置舵机ITEM不可拖动')
            self.show_info('设置可拖动成功')

    def unload_all_steerings(self):
        items = self.get_selected_item(flag=True)

        for item in items:
            item.unload()
        self.show_info('所有舵机已掉电')

    def deviation_to_zero(self):
        items = self.get_selected_item()
        if not items:
            self.show_info('请先选择舵机')
            return

        flag = QMessageBox.question(self, '警告', '您已选的舵机是否全部偏差归零',
                                    QMessageBox.Yes | QMessageBox.No,
                                    QMessageBox.Yes)
        if flag == QMessageBox.No:
            return
        for item in items:
            item.spinBox.setValue(0)
            item.deviation_value_change()

    def batch_steering_data(self):
        items = self.get_selected_item()
        if not items:
            self.show_info('请先选择舵机')

        degree = int(self.lineEdit.text())
        for item in items:
            item.lineEdit_3.setText(str(degree))
            item.degree_change()
        pass

    def get_selected_item(self, flag=False):
        _ = []
        for item in self.widget.findChildren(QtItems):
            if flag:
                _.append(item)
            else:
                if item.is_selected:
                    _.append(item)
        return _

    def get_steering_degree(self):
        items = self.get_selected_item()
        if not items:
            self.show_info('请先选择舵机')
        for item in items:
            item.get_degree()

    def get_steering_deviation(self):
        items = self.get_selected_item()
        if not items:
            self.show_info('请先选择舵机')
        for item in items:
            item.get_deviation_value()

    def save_steering_deviation(self):
        items = self.get_selected_item()
        if not items:
            self.show_info('请先选择舵机')
            return
        flag = QMessageBox.question(self, '警告', '您已选的舵机是否保存舵机偏差',
                                    QMessageBox.Yes | QMessageBox.No,
                                    QMessageBox.Yes)
        if flag == QMessageBox.No:
            return
        for item in items:
            item.save_deviation_value()

    def closeEvent(self, event):
        self.widget.save_item_pos()
        print('QCloseEvent')


class RunActionGroup(QThread):
    end_run_sign = pyqtSignal()

    def run(self):
        for action in self.action_group_data:
            print(action['name'])
            for steering_dict in action['steerings_data']:
                if not self.is_active:
                    self.end_run_sign.emit()
                    return
                print(steering_dict)
            time.sleep(action['sleep_time'] / 1000)

        if self.is_loop:
            self.run()
        else:
            print(self.__class__.__name__, '已结束')
            self.end_run_sign.emit()

    def __del__(self):
        print(self.__class__.__name__, '已被销毁')


if __name__ == '__main__':
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    app = QApplication(sys.argv)
    ex = QtMain()
    ex.show()
    app.exec_()
