from typing import List
import datetime

from .utils import get_h_m_s

# Header is a message header byte
CMD_HEADER: int = 0x9A

def add_bcc(cmd: List[int]):
    """Compute BCC (= Block Checking Charactor) and append to command sequence.
    
    Returns:
        list of binary data with BCC code.
    """
    check: int = 0x00
    for b in cmd:
        check = check ^ b
    cmd.append(check)
    return cmd

class CmdTemplate(object):
    """ Base class for TSDN command  serial communication.
    """
    name = "none"
    cmd_code: int = None 
    response_code: int = None
    response_param_size: int = None
    pyload: bytes = None
    
    def __init__(self):
        pass

    @property
    def response_size(self):
        return self.response_param_size + 3

    def encode(self):
        raise NotImplementedError()

    def validate_response(self, response: bytes):
        assert response[1] == self.response_code, f"respose_code={self.response_code}, response={response}"

    def decode(self, response: bytes) -> dict[str:any]:
        raise NotImplementedError()
    
    def pformat(self, outputs: dict):
        return f"{self.name}:: {outputs}"

class SetDeviceTime(CmdTemplate):
    name = "SetDeviceTime"
    cmd_code = 0x11
    response_code = 0x8F
    response_param_size = 1

    def encode(self, ts: datetime.datetime=None):
        if ts is None:
            ts = datetime.datetime.now()

        ms = int(ts.microsecond // 1e3)
        cmd = [
            CMD_HEADER,
            self.cmd_code,
            ts.year - 2000,
            ts.month,
            ts.day,
            ts.hour,
            ts.minute,
            ts.second,
            (ms%256),
            (ms//256),
        ]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {"status": response[2]}
        return data

class GetDeviceTime(CmdTemplate):
    name = "DeviceTime"
    cmd_code = 0x12
    response_code = 0x92
    response_param_size = 8

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)

        return bytes(cmd)

    def decode(self, response: bytes) -> datetime.datetime:
        self.validate_response(response)
            
        # -- Decode --
        ts = datetime.datetime(
            (response[2] + 2000), # year
            response[3], # month
            response[4], # day
            hour=response[5],
            minute=response[6],
            second=response[7],
            microsecond=(int.from_bytes(response[8:10], "little") * 1000),
        )
        data = {
            "status": 0,
            "time": ts,
        }
        return data


class StartRecording(CmdTemplate):
    """計測開始 / 計測予約 (Command) & 計測時刻応答 (Response)

    (Command) 計測の開始または開始時刻及び終了時刻の設定を行う.
    開始時刻及び終了時刻は, 相対時間または絶対時間の指定が可能.

    (Response) 計測時刻の設定状態と設定された計測の開始時刻及び終了時刻を応答する.

    Note:
        現状では，開始時刻は相対時間による指定，終了時刻の指定なしのみサポート
    """
    name = "StartRecording"
    cmd_code = 0x13
    response_code = 0x93
    response_param_size = 13

    def encode(
        self,
        mode:int = 0,
        td_start: datetime.timedelta=None,
        td_end: datetime.timedelta=None,
    ):
        assert mode == 0
        assert td_end is None
        if td_start is None:
            td_start = datetime.timedelta(seconds=5)

        cmd = [CMD_HEADER, self.cmd_code]
        
        # Start Time
        # NOTE: For mode=0 (relative time mode), year, month, day will be ignored.
        h, m, s = get_h_m_s(td_start) 
        cmd += [mode, 0, 1, 1, h, m, s] # mode, year, month, day, hour, minute, second
        
        # End Time (free run)
        # NOTE: For mode=0 (relative time mode), year, month, day will be ignored.
        cmd += [mode, 0, 1, 1, 0, 0, 0] # mode, year, month, day, hour, minute, second
        
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        
        status = response[2]
        ts_start = datetime.datetime(
            (response[3] + 2000), # year
            response[4], # month
            response[5], # day
            response[6], # hour
            response[7], # minute
            response[8], # second
        )
        ts_end = datetime.datetime(
            (response[9] + 2000), # year
            response[10], # month
            response[11], # day
            response[12], # hour
            response[13], # minute
            response[14], # second
        )

        outputs = {
            "status": 0,
            "recoding_status": status,
            "start": ts_start,
            "end": ts_end,
        }
        return outputs


class StopRecording(CmdTemplate):
    name = "StopRecording"
    cmd_code = 0x15
    response_code = 0x8F
    response_param_size = 1

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response: bytes) -> dict:
        self.validate_response(response)
        outputs = {"status": response[2]}
        return outputs

class SetAgsMethod(CmdTemplate):
    """加速/角速度計測設定取得
    """
    name = "SetAgsMethod"
    cmd_code = 0x16
    response_code = 0x8F
    response_param_size = 1

    def encode(self, interval=None, send_freq=None, record_freq=None):
        """
        Args:
            interval (int): 計測の実施有無及び計測周期を設定.
                0:計測 OFF, 1~255: 計測周期 (1ms 単位指定)
            send_freq (int): 計測データ送信の実施有無及び送信時の平均回数を設定
                0:送信しない, 1~255: 平均回数
            record_freq (int): 計測データ記録の実施有無及び記録時の平均回数を設定
                0:記録しない, 1~255: 平均回数
        """
        assert interval >= 0 and interval < 256
        assert send_freq >= 0 and send_freq < 256
        assert record_freq >= 0 and record_freq < 256

        cmd = [
            CMD_HEADER,
            self.cmd_code,
            interval,
            send_freq,
            record_freq,
        ]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {
            "status": response[2],
        }
        return data


class GetAgsMethod(CmdTemplate):
    """加速/角速度計測設定
    """
    name = "AgsMethod"
    cmd_code = 0x17
    response_code = 0x97
    response_param_size = 3

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)

        data = {
            "status": 0,
            "interval": response[2],
            "send_freq": response[3],
            "record_freq": response[4],
        }
        return data

class SetGeoMagneticMethod(CmdTemplate):
    """地磁気計測設定
    """
    name = "SetGeoMagneticMethod"
    cmd_code = 0x18
    response_code = 0x8F
    response_param_size = 1

    def encode(self, interval=None, send_freq=None, record_freq=None):
        """
        Args:
            interval (int): 計測の実施有無及び計測周期を設定.
                0:計測 OFF, 1~255: 計測周期 (1ms 単位指定)
            send_freq (int): 計測データ送信の実施有無及び送信時の平均回数を設定
                0:送信しない, 1~255: 平均回数
            record_freq (int): 計測データ記録の実施有無及び記録時の平均回数を設定
                0:記録しない, 1~255: 平均回数
        """
        assert interval >= 0 and interval < 256
        assert send_freq >= 0 and send_freq < 256
        assert record_freq >= 0 and record_freq < 256

        cmd = [
            CMD_HEADER,
            self.cmd_code,
            interval,
            send_freq,
            record_freq,
        ]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {
            "status": response[2],
        }
        return data

class GetGeoMagneticMethod(CmdTemplate):
    """地磁気計測設定取得
    """
    name = "GeoMagneticMethod"
    cmd_code = 0x19
    response_code = 0x99
    response_param_size = 3

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {
            "status": 0,
            "interval": response[2],
            "send_freq": response[3],
            "record_freq": response[4],
        }
        return data


class SetPresMethod(CmdTemplate):
    """気圧計測設定
    """
    name = "SetPresMethod"
    cmd_code = 0x1A
    response_code = 0x8F
    response_param_size = 1

    def encode(self, interval=None, send_freq=None, record_freq=None):
        """
        Args:
            interval (int): 計測の実施有無及び計測周期を設定.
                0:計測 OFF, 1~255: 計測周期 (1ms 単位指定)
            send_freq (int): 計測データ送信の実施有無及び送信時の平均回数を設定
                0:送信しない, 1~255: 平均回数
            record_freq (int): 計測データ記録の実施有無及び記録時の平均回数を設定
                0:記録しない, 1~255: 平均回数
        """
        assert interval >= 0 and interval < 256
        assert send_freq >= 0 and send_freq < 256
        assert record_freq >= 0 and record_freq < 256

        cmd = [
            CMD_HEADER,
            self.cmd_code,
            interval,
            send_freq,
            record_freq,
        ]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {"status": response[2]}
        return data

class GetPresMethod(CmdTemplate):
    """地磁気計測設定取得
    """
    name = "PresMethod"
    cmd_code = 0x1B
    response_code = 0x9B
    response_param_size = 3

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)

        data = {
            "status": 0,
            "interval": response[2],
            "send_freq": response[3],
            "record_freq": response[4],
        }
        return data


class SetBattMethod(CmdTemplate):
    """バッテリ電圧計測設定
    """
    name = "SetBattMethod"
    cmd_code = 0x1C
    response_code = 0x8F
    response_param_size = 1

    def encode(self, send=None, record=None):
        """
        Args:
            is_send (int): 計測データ送信の実施有無を設定
                0:送信しない, 1:送信する.
            is_record (int): 計測データ記録の実施有無を選択
                0:記録しない、1:記録する
        """
        assert send in [0, 1]
        assert record in [0, 1]

        cmd = [
            CMD_HEADER,
            self.cmd_code,
            send,
            record,
        ]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {
            "status": response[2],
        }
        return data

class GetBattMethod(CmdTemplate):
    """バッテリ電圧計測設定取得
    """
    name = "BattMethod"
    cmd_code = 0x1D
    response_code = 0x9D
    response_param_size = 2

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)

        data = {
            "status": 0,
            "send": response[2],
            "record": response[3],
        }
        return data



class SetExtInputSetting(CmdTemplate):
    """外部拡張端子計測&エッジデータ出力設定
    """
    name = "SetExtInputSetting"
    cmd_code = 0x1E
    response_code = 0x8F
    response_param_size = 1

    def encode(self, interval=None, send=None, record=None, edge_send=None, edge_record=None):
        """
        Args:
            interval (int): 計測 OFF or 計測周期
                0: 計測 OFF, 2: 計測周期 (1ms～255ms, 1ms 単位指定)
            send (int): 計測データ送信設定
                0: 送信しない, 1: 平均回数 (1～255回)
            record (int): 計測データ記録設定
                0: 記録しない, 1: 平均回数 (1～255回)
            edge_send (int): エッジ検出データ送信設定.
                外部拡張端子入力及びオプションスイッチからのエッジ検出
                データの送信実施有無を選択.
                0: 送信しない, 1: 送信する.
            edge_record (int): エッジ検出データ記録設定
                外部拡張端子入力及びオプションスイッチからのエッジ検出
                データの記録実施有無を選択.
                0:記録しない, 1:記録する.
        """
        assert interval >= 0 and interval < 256
        assert send >= 0 and send < 256
        assert record >= 0 and record < 256
        assert edge_send in [0, 1]
        assert edge_record in [0, 1]
        
        cmd = [
            CMD_HEADER,
            self.cmd_code,
            interval,
            send,
            record,
            edge_send,
            edge_record,
        ]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {
            "status": response[2],
        }
        return data

class GetExtInputSetting(CmdTemplate):
    """外部拡張端子計測&エッジデータ出力設定取得
    """
    name = "ExtInputSetting"
    cmd_code = 0x1F
    response_code = 0x9F
    response_param_size = 5

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)

        data = {
            "status": 0,
            "interval": response[2],
            "send": response[3],
            "record": response[4],
            "edge_send": response[5],
            "edge_record": response[6],
        }
        return data

class SetI2CSetting(CmdTemplate):
    """外部拡張 I2C 通信設定
    """
    name = "SetI2CSetting"
    cmd_code = 0x20
    response_code = 0x8F
    response_param_size = 1

    def encode(self, interval=None, send=None, record=None):
        """
        Args:
            interval (int): 計測 OFF or 計測周期
                0: 計測 OFF, 2: 計測周期 (1ms～255ms, 1ms 単位指定)
            send (int): 計測データ送信設定
                0: 送信しない, 1: 平均回数 (1～255回)
            record (int): 計測データ記録設定
                0: 記録しない, 1: 平均回数 (1～255回)
        """
        assert interval >= 0 and interval < 256
        assert send >= 0 and send < 256
        assert record >= 0 and record < 256
        
        cmd = [
            CMD_HEADER,
            self.cmd_code,
            interval,
            send,
            record,
        ]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {
            "status": response[2],
        }
        return data

class GetI2CSetting(CmdTemplate):
    """外部拡張 I2C 通信取得

    外部拡張 I2C 通信の計測に関する設定を取得する。
    """
    name = "I2CSetting"
    cmd_code = 0x21
    response_code = 0xA1
    response_param_size = 3

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)

        data = {
            "status": 0,
            "interval": response[2],
            "send": response[3],
            "record": response[4],
        }
        return data

class SetAccRange(CmdTemplate):
    """加速度センサ計測レンジ設定
    """
    name = "SetAccRange"
    cmd_code = 0x22
    response_code = 0x8F
    response_param_size = 1
    translation = {
        0: "+/-2G",
        1: "+/-4G",
        2: "+/-8G",
        3: "+/-16G",
    }

    def encode(self, mode=None):
        """
        Args:
            mode (Literal[0, 1, 2, 3]): 加速度センサの計測レンジを設定
                0: +/-2G, 1:+/-4G, 2: +/-8G, 3: +/-16G
        """
        assert mode in [0, 1, 2, 3]
        
        cmd = [CMD_HEADER, self.cmd_code, mode]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {"status": response[2]}
        return data

class GetAccRange(CmdTemplate):
    """加速度センサ計測レンジ設定取得
    """
    name = "AccRange"
    cmd_code = 0x23
    response_code = 0xA3
    response_param_size = 1
    translation = {
        0: "+/-2G",
        1: "+/-4G",
        2: "+/-8G",
        3: "+/-16G",
    }

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        assert response[1] == self.response_code
        data = {
            "status": 0,
            "mode": response[2],
            "mode_display": self.translation.get(response[2]),
        }
        return data



class ClearMemoery(CmdTemplate):
    """計測データ記録クリア
    """
    name = "ClearMemory"
    cmd_code = 0x35
    response_code = 0x8F
    response_param_size = 1

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)

        data = {
            "status": response[2],
        }
        return data

class GetMemEntryCount(CmdTemplate):
    """計測データ記録エントリ件数取得
    
    """
    name = "MemEntryCount"
    cmd_code = 0x36
    response_code = 0xB6
    response_param_size = 1

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)

        data = {
            "status": 0,
            "num_entry": response[2],
        }
        return data

class GetEntryInfo(CmdTemplate):
    """計測データ記録エントリ取得
    
    Note:
        加速角速度データは, 1計測当たり2レコードとなる. その他は1レコード.
    """
    name = "EntryInfo"
    cmd_code = 0x37
    response_code = 0xB7
    response_param_size = 24

    def encode(self, entry_index: int=None):
        assert entry_index > 0 and entry_index <= 80

        cmd = [CMD_HEADER, self.cmd_code, entry_index]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)

        ts_start = datetime.datetime(
            (response[2] + 2000), # year
            response[3], # month
            response[4], # day
            hour=response[5], # hour
            minute=response[6], # minute
            second=response[7], # second
            microsecond=(int.from_bytes(response[8:10], "little") * 1000),
        )
        
        data = {
            "status": 0,
            "ts_start": ts_start.strftime("%Y-%m-%d %H-%M-%S.%f"), 
            "num_records":int.from_bytes(response[10:14], "little"),
            "ags_interval": response[14],
            "geo_interval": response[15],
            "pres_interval": response[16],
            "ext_interval": response[17],
            "i2c_interval": response[18],
            "ags_record_freq": response[19],
            "geo_record_freq": response[20],
            "pres_record_freq": response[21],
            "batt_record_freq": response[22],
            "ext_record_freq": response[23],
            "i2c_record_freq": response[24],
            "edge_event": response[25],
        }
        return data


class GetEntryDetail(CmdTemplate):
    """計測データ記録エントリ詳細取得
    """
    name = "EntryDetail"
    cmd_code = 0x38
    response_code = 0xB8
    response_param_size = 60

    def encode(self, entry_index: int=None):
        assert entry_index > 0 and entry_index <= 80

        cmd = [CMD_HEADER, self.cmd_code, entry_index]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)        
        data = {
            "status": 0,
            "acc_range": response[2], 
            "acc_target_x": response[3], # 加速度補正目標設定 
            "acc_target_y": response[4],
            "acc_target_z": response[5],
            "acc_offset_x": int.from_bytes(response[6:10], "little"),
            "acc_offset_y": int.from_bytes(response[10:14], "little"),
            "acc_offset_z": int.from_bytes(response[14:18], "little"),
            "gyro_range": response[18], 
            "gyro_target_x": response[19], # 角速度補正目標設定 
            "gyro_target_y": response[20],
            "gyro_target_z": response[21],
            "gyro_offset_x": int.from_bytes(response[22:26], "little"),
            "gyro_offset_y": int.from_bytes(response[26:30], "little"),
            "gyro_offset_z": int.from_bytes(response[30:34], "little"),
            "geo_callibration_x": int.from_bytes(response[34:38], "little"),
            "geo_callibration_y": int.from_bytes(response[38:42], "little"),
            "geo_callibration_z": int.from_bytes(response[42:46], "little"),
            "I2C_comm_speed": response[46], 
            "I2C_slave": response[47], # 加速度補正目標設定 
            "I2C_send_data_size": response[48],
            "I2C_send_data": list(response[49:57]),
            "I2C_recv_data_size": response[57],
            "I2C_mode_ch1": response[58],
            "I2C_mode_ch2": response[59],
            "I2C_mode_ch3": response[60],
            "I2C_mode_ch4": response[61],
        }
        return data

class ReadMemData(CmdTemplate):
    """計測データ記録メモリ読み出し
    """
    name = "ReadMemData"
    cmd_code = 0x39
    response_code = 0xB9
    response_param_size = 1

    def encode(self, entry_index):
        assert entry_index > 0 and entry_index <= 80, f"entry_index={entry_index}"

        cmd = [CMD_HEADER, self.cmd_code, entry_index]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {
            "status": 0,
        }
        return data

class GetFreeMemSize(CmdTemplate):
    """計測データ記録メモリ残容量取得
    """
    name = "FreeMemSize"
    cmd_code = 0x3A
    response_code = 0xBA
    response_param_size = 5

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)

        data = {
            "status": 0,
            "num_free_entries": response[2],
            "num_free_records": int.from_bytes(response[2:6], "little"),
        }
        return data

class GetBattStatus(CmdTemplate):
    """バッテリ状態取得
    """
    name = "BattStatus"
    cmd_code = 0x3B
    response_code = 0xBB
    response_param_size = 3

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {
            "status": 0,
            "voltage": int.from_bytes(response[2:4], "little"),
            "voltage__unit": "0.01V",
            "remaining": response[4],
            "remaining__unit": "%",
        }
        return data


class GetDeviceStatus(CmdTemplate):
    """動作状態取得
    
    Status Code:
    * 0: USB 接続中コマンドモード
    * 1: USB 接続中計測モード
    * 2: Bluetooth 接続中コマンドモード
    * 3: Bluetooth 接続中計測モード
    """
    name = "DeviceStatus"
    cmd_code = 0x3C
    response_code = 0xBC
    response_param_size = 1
    translator = {
        0: "USB Connected. Command mode.", # "USB 接続中コマンドモード",
        1: "USB connected. Recording mode.", # "USB 接続中計測モード",
        2: "Bluetooth conneted. Command mode.", # "Bluetooth 接続中コマンドモード",
        3: "Bluetooth connected. Recording mode.", # "Bluetooth 接続中計測モード"
    }

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)

        data = {
            "status": 0,
            "device_status": response[2],
            "device_status_display": self.translator.get(response[2]),
        }
        return data



class SetAutoPowerOffTime(CmdTemplate):
    """オートパワーオフ時間設定

    BT 接続待機モード時のオートパワーオフ時間を設定する.
    """
    name = "SetAutoPowerOffTime"
    cmd_code = 0x50
    response_code = 0x8F
    response_param_size = 1

    def encode(self, minutes):
        """
        Args:
            minutes (int): オートパワーオフ時間
                0: オートパワーオフ機能無効, 1-20: オートパワーオフ時間 (分)
        """
        assert minutes >= 0 and minutes <= 20
        
        cmd = [CMD_HEADER, self.cmd_code, minutes]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {"status": response[2]}
        return data

class GetAutoPowerOffTime(CmdTemplate):
    """オートパワーオフ時間設定取得
    """
    name = "AutoPowerOffTime"
    cmd_code = 0x51
    response_code = 0xD1
    response_param_size = 1

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)

        data = {
            "status": 0,
            "minutes": response[2],
        }
        return data


class StopReadMemData(CmdTemplate):
    """計測データ記録読み出し中断 & 計測データ記録メモリ読み出し完了応答
    
    Response=指定されたエントリ番号の計測データが全て送信.もしくは送信中断されたことを示す応答.
    """
    name = "StopReadMemData"
    cmd_code = 0x54
    response_code = 0xB9
    response_param_size = 1

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {
            "status": 0,
        }
        return data



class SetQuaternionSetting(CmdTemplate):
    """クオータニオン計測設定
    """
    name = "SetQuaternionSetting"
    cmd_code = 0x55
    response_code = 0x8F
    response_param_size = 1

    def encode(self, interval=None, send=None, record=None):
        """
        Args:
            interval (int): 計測 OFF or 計測周期
                0: 計測 OFF, 2: 計測周期 (1ms～255ms, 1ms 単位指定, 5ms刻み)
            send (int): 計測データ送信設定
                0: 送信しない, 1: 平均回数 (1～255回)
            record (int): 計測データ記録設定
                0: 記録しない, 1: 平均回数 (1～255回)
        """
        assert interval >= 0 and interval < 256
        assert send >= 0 and send < 256
        assert record >= 0 and record < 256
        
        cmd = [
            CMD_HEADER,
            self.cmd_code,
            interval,
            send,
            record,
        ]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {
            "status": response[2],
        }
        return data

class GetQuaternionSetting(CmdTemplate):
    """クオータニオン計測設定取得
    """
    name = "QuaternionSetting"
    cmd_code = 0x56
    response_code = 0xD6
    response_param_size = 3

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {
            "status": 0,
            "interval": response[2],
            "send": response[3],
            "record": response[4],
        }
        return data


class SetExt16ADSetting(CmdTemplate):
    """拡張 16bitAD 計測設定
    """
    name = "SetExt16ADSetting"
    cmd_code = 0x59
    response_code = 0x8F
    response_param_size = 7

    def encode(self, interval=None, send=None, record=None, mode_ch1=None, mode_ch2=None, mode_ch3=None, mode_ch4=None):
        """
        Args:
            interval (int): 計測 OFF or 計測周期
                0: 計測 OFF, 2: 計測周期 (1ms～255ms, 1ms 単位指定)
            send (int): 計測データ送信設定
                0: 送信しない, 1: 平均回数 (1～255回)
            record (int): 計測データ記録設定
                0: 記録しない, 1: 平均回数 (1～255回)
            mode_ch1 (int): 拡張 16bitAD1ch モード
                0:未使用, 1～12: ゲイン 1～12,
            mode_ch2 (int): 拡張 16bitAD2ch モード
            mode_ch3 (int): 拡張 16bitAD3ch モード
            mode_ch4 (int): 拡張 16bitAD4ch モード
        """
        assert interval >= 0 and interval < 256
        assert send >= 0 and send < 256
        assert record >= 0 and record < 256
        assert mode_ch1 in [0, 1, 2, 3, 4, 6, 8, 12]
        assert mode_ch2 in [0, 1, 2, 3, 4, 6, 8, 12]
        assert mode_ch3 in [0, 1, 2, 3, 4, 6, 8, 12]
        assert mode_ch4 in [0, 1, 2, 3, 4, 6, 8, 12]
        
        cmd = [
            CMD_HEADER,
            self.cmd_code,
            interval,
            send,
            record,
            mode_ch1,
            mode_ch2,
            mode_ch3,
            mode_ch4,
        ]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {
            "status": response[2],
        }
        return data

class GetExt16ADSetting(CmdTemplate):
    """拡張 16bitAD 計測設定取得
    """
    name = "Ext16ADSetting"
    cmd_code = 0x5A
    response_code = 0xDA
    response_param_size = 7

    def encode(self):
        cmd = [CMD_HEADER, self.cmd_code, 0x00]
        cmd = add_bcc(cmd)
        return bytes(cmd)

    def decode(self, response):
        self.validate_response(response)
        data = {
            "status": 0,
            "interval": response[2],
            "send": response[3],
            "record": response[4],
            "mode_ch1": response[5],
            "mode_ch2": response[6],
            "mode_ch3": response[7],
            "mode_ch4": response[8],
        }
        return data



# -------------------------------------------------------------------------------------
# == Event ==

class AgsDataEvent(CmdTemplate):
    """加速度角速度計測データ通知
    
    Data Field:
    * TickTime[4Byte] = 計測年月日の 00:00:00:000 からの経過時間(ms 単位)
    * 加速度データ X [3Byte] = -160000～160000(0.1mg 単位)
    * 加速度データ Y [3Byte] = -160000～160000(0.1mg 単位)
    * 加速度データ Z [3Byte] = -160000～160000(0.1mg 単位)
    * 角速度データ X [3Byte] = -200000～200000(0.01dps 単位)
    * 角速度データ Y [3Byte] = -200000～200000(0.01dps 単位)
    * 角速度データ Z [3Byte] = -200000～200000(0.01dps 単位)
    """
    name = "AgsDataEvent"
    cmd_code = None
    response_code = 0x80
    response_param_size = 22

    def decode(self, response: bytes) -> dict:
        self.validate_response(response)
        
        # -- Timestamp --
        ts = int.from_bytes(response[2:6], "little")

        # -- Sensor Readings --
        acc = [
            int.from_bytes(response[6:9], "little", signed=True), # Acc-X
            int.from_bytes(response[9:12], "little", signed=True), # Acc-Y
            int.from_bytes(response[12:15], "little", signed=True), # Acc-Z
        ]
        gyro = [
            int.from_bytes(response[15:18], "little", signed=True), # Gyro-X
            int.from_bytes(response[18:21], "little", signed=True), # Gyro-Y
            int.from_bytes(response[21:24], "little", signed=True), # Gyro-Z
        ]

        outputs = {
            "ts": ts,
            "acc": acc,
            "gyro": gyro,
        }
        return outputs

class BattEvent(CmdTemplate):
    """バッテリ電圧計測データ通知
    """
    name = "BattEvent"
    cmd_code = None
    response_code = 0x83
    response_param_size = 7

    def decode(self, response: bytes) -> dict:
        self.validate_response(response)
        outputs = {
            "ts": int.from_bytes(response[2:6], "little"),
            "voltage": int.from_bytes(response[6:8], "little") / 100, # [Unit: V]
            "remaining": response[8], # [Unit: %]
        }
        return outputs

class RecodingErrorEvent(CmdTemplate):
    """計測エラー通知
    
    計測中にエラーが発生したことを通知する。
    
    Note:
        エラー要因[1Byte] = 0x80 or 0x81 or 0x82 or 0x86 or 0x8A or 0x8B or 0x8C
        * 0x80:加速度角速度センサ計測異常発生
        * 0x81:地磁気センサ計測異常発生
        * 0x82:気圧センサ計測異常発生
        * 0x86:外部拡張 I2C 計測異常発生
        * 0x8A:クオータニオン計測異常発生
        * 0x8B:外部拡張 I2C 計測 2 異常発生
        * 0x8C：拡張 16bitAD 計測異常発生
    """
    name = "RecongingErrorEvent"
    cmd_code = None
    response_code = 0x87
    response_param_size = 5
    translator = {
        0x80: "加速度角速度センサ計測異常発生",
        0x81: "地磁気センサ計測異常発生",
        0x82: "気圧センサ計測異常発生",
        0x86: "外部拡張 I2C 計測異常発生",
        0x8A: "クオータニオン計測異常発生",
        0x8B: "外部拡張 I2C 計測 2 異常発生",
        0x8C: "拡張 16bitAD 計測異常発生",
    }

    def decode(self, response: bytes) -> dict:
        self.validate_response(response)

        outputs = {
            "time": int.from_bytes(response[2:6], "little"),
            "reason": response[6],
            "reason_display": self.translator[response[6]],
        }
        return outputs

class RecodingStartedEvent(CmdTemplate):
    """計測開始通知
    
    計測開始を通知する. 本通知は計測記録メモリの対象としない.
    """
    name = "RecongingStartedEvent"
    cmd_code = None
    response_code = 0x88
    response_param_size = 1

    def decode(self, response: bytes) -> dict:
        self.validate_response(response)

        outputs = {
            "recording": "started",
        }
        return outputs


class RecodingStoppedEvent(CmdTemplate):
    """計測終了通知

    計測が終了したことを通知する. 本通知は計測記録メモリの対象としない.

    計測終了ステータス:
    * 0: 計測停止コマンド及び終了時刻による終了
    * 1: OptionSW 操作による終了
    * 2: 計測記録メモリフル終了
    * 3: バッテリ残量低下による終了
    * 100: 開始エラー(同時計測記録可能量オーバー、計測対象無し)
    * 101: 開始エラー(拡張 I2C 異常)
    """
    name = "RecongingStoppedEvent"
    cmd_code = None
    response_code = 0x89
    response_param_size = 1

    def decode(self, response: bytes) -> dict:
        self.validate_response(response)

        outputs = {
            "recording": "stopped",
            "status": response[2],
        }
        return outputs


class QuaternionEvent(CmdTemplate):
    """加速度角速度計測データ通知
    
    Data Field:
    * TickTime[4Byte] = 計測年月日の 00:00:00:000 からの経過時間(ms 単位)
    * クオータニオンデータ W [2Byte] = -10000～10000 (0.0001 単位)
    * クオータニオンデータ X [2Byte] = -10000～10000 (0.0001 単位)
    * クオータニオンデータ Y [2Byte] = -10000～10000 (0.0001 単位)
    * クオータニオンデータ Z [2Byte] = -10000～10000 (0.0001 単位)
    * 加速度データ X [3Byte] = -160000～160000(0.1mg 単位)
    * 加速度データ Y [3Byte] = -160000～160000(0.1mg 単位)
    * 加速度データ Z [3Byte] = -160000～160000(0.1mg 単位)
    * 角速度データ X [3Byte] = -200000～200000(0.01dps 単位)
    * 角速度データ Y [3Byte] = -200000～200000(0.01dps 単位)
    * 角速度データ Z [3Byte] = -200000～200000(0.01dps 単位)
    """
    name = "QuaternionEvent"
    cmd_code = None
    response_code = 0x8A
    response_param_size = 30

    def decode(self, response: bytes) -> dict:
        self.validate_response(response)
        
        # -- Timestamp --
        ts = int.from_bytes(response[2:6], "little")

        # -- Sensor Readings --
        qua = [
            int.from_bytes(response[6:8], "little", signed=True), # Qua-W
            int.from_bytes(response[8:10], "little", signed=True), # Qua-X
            int.from_bytes(response[10:12], "little", signed=True), # Qua-Y
            int.from_bytes(response[12:14], "little", signed=True), # Qua-Z
        ]
        acc = [
            int.from_bytes(response[14:17], "little", signed=True), # Acc-X
            int.from_bytes(response[17:20], "little", signed=True), # Acc-Y
            int.from_bytes(response[20:23], "little", signed=True), # Acc-Z
        ]
        gyro = [
            int.from_bytes(response[23:26], "little", signed=True), # Gyro-X
            int.from_bytes(response[26:29], "little", signed=True), # Gyro-Y
            int.from_bytes(response[29:31], "little", signed=True), # Gyro-Z
        ]

        outputs = {
            "ts": ts,
            "qua": qua,
            "acc": acc,
            "gyro": gyro,
        }
        return outputs