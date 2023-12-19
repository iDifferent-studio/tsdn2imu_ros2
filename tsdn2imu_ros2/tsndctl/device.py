from ctypes import resize
from datetime import datetime
import time
import serial
import threading
from pprint import pprint

from . import commands as tsndcmd
from logging import getLogger


def split_response(res):
    """
    Note:
        Implement unit-test
    """
    res_out = []
    ind_head = 0
    for i in range(len(res)):
        if res[i] == 0x9a and i > 0:
            res_out.append(res[ind_head:i])
            ind_head = i
    return res_out


class TSND151(object):
    def __init__(
        self,
        name: str =None,
        port: str=None,
        logdir: str=None,
        baudrate: int =9600,
        timeout: float=5,
        conf: dict=None,
        logger = None,
    ) -> None:
        self.name = name
        self.port = port
        self.logdir = logdir
        self.conf = conf
        self.timeout = timeout
        self.is_running = False
        self.is_thread_running = False
        self.logger = None
        assert timeout is not None
        if self.logger is None:
            self.logger = getLogger(f"tsndctl.TSND151.{self.name}")

        # Setup serial port
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.thread = threading.Thread(target=self.event_listener)

    def start_event_listener(self):
        self.is_running = True
        self.is_thread_started = True
        self.thread.start()
        
    def stop_event_listener(self):
        self.is_running = False
        time.sleep(self.timeout * 2)
        if self.is_thread_running:
            self.thread.join()
            self.is_thread_running= False

    def terminate(self):
        self.logger.info(f"Terminate serial connection to the device [{self.name}]")
        self.is_running = False
        self.ser.close()

    def send(self, msg: bytes):
        while True:
            if self.ser.out_waiting == 0:
                break
        self.ser.write(msg)
        self.ser.flush()
        
    def listen(self, num_bytes) -> bytes:
        response = None
        while True:
            if self.ser.in_waiting > 0:
                response = self.ser.read(num_bytes)
                break
        return response

    def event_listener(self):
        handler = {
            0x80: tsndcmd.AgsDataEvent(),
            0x88: tsndcmd.RecodingStartedEvent(),
            0x89: tsndcmd.RecodingStoppedEvent(),
            0x8A: tsndcmd.QuaternionEvent(),
            0xB9: tsndcmd.ReadMemData(),
            0x8F: tsndcmd.StopRecording(), # FIXME: Use dummy decoder.
        }
        pprint(handler)
        
        server_on = True
        while server_on:
            if self.is_running == False:
                break
            if self.ser.in_waiting > 0:
                response = self.ser.readline()
                if len(response) == 0:
                    self.logger.warning(f"Timeout ({response})")
                    continue
                
                # TODO: (1) Split by \x9a
                responses = split_response(response)
                # TODO: (2) decode each segment
                for i, res in enumerate(responses):
                    if len(res) < 2:
                        self.logger.warning(f"ShortResponseMsg (response={str(res)})")
                        continue
                    if res[0] != 0x9a:
                        self.logger.warning(f"BrokenResponseMsg (response={str(res)})")
                        continue
                    cmd = handler.get(res[1], None) 
                    if cmd is None:
                        self.logger.warning(f"UnknownResponse (response={str(res)})")
                        continue
                    elif cmd.name == "ReadMemData":
                        server_on = False # FIXME: ??
                        self.logger.warning(f"ReadMemDataResponse is detected! (response={str(res)})")
                        # break
                    else:
                        res = cmd.decode(res)
                        self.logger.info(cmd.pformat(res))
                if server_on is False:
                    break
        self.logger.info("Stop server")
        
    def process_command(
        self,
        cmd: tsndcmd.CmdTemplate,
        params: dict=None,
        num_tot: int=0,
        num_ok: int=0,
    ):
        if params is None:
            msg = cmd.encode()
        else:
            msg = cmd.encode(**params)
        
        self.send(msg)
        response = self.listen(cmd.response_size)
        response = cmd.decode(response)
        
        num_tot += 1
        if response.get("status", 1) == 0:
            num_ok += 1
        return response, num_tot, num_ok

    def init_device(self):
        self.set_device_params()
        self.get_device_status()

    def set_device_params(self):
        # TODO: Refactoring is Needed!!
        self.logger.debug("== Set Device Params ==")
        num_tot, num_ok = 0, 0
        
        # -- Send Device Time --
        cmd = tsndcmd.SetDeviceTime()
        response, num_tot, num_ok = self.process_command(
            cmd, num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.debug(cmd.pformat(response))
        time.sleep(1)
        
        # -- Set Ags Method --
        cmd = tsndcmd.SetAgsMethod()
        response, num_tot, num_ok = self.process_command(
            cmd,
            params={"interval": 33, "send_freq": 30, "record_freq": 1},
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.debug(cmd.pformat(response))
        time.sleep(1)

        # -- Set GeoMagnetic Method --
        cmd = tsndcmd.SetGeoMagneticMethod()
        response, num_tot, num_ok = self.process_command(
            cmd,
            params={"interval": 0, "send_freq": 0, "record_freq": 0},
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.debug(cmd.pformat(response))
        time.sleep(1)

        # -- Set Pressure Method --
        cmd = tsndcmd.SetPresMethod()
        response, num_tot, num_ok = self.process_command(
            cmd,
            params={"interval": 0, "send_freq": 0, "record_freq": 0},
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.debug(cmd.pformat(response))
        time.sleep(1)

        # -- Set Battery Method --
        cmd = tsndcmd.SetBattMethod()
        response, num_tot, num_ok = self.process_command(
            cmd,
            params={"send": 0, "record": 0},
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.debug(cmd.pformat(response))
        time.sleep(1)
        
        # -- Set/Get ExtInput Setting --
        cmd = tsndcmd.SetExtInputSetting()
        response, num_tot, num_ok = self.process_command(
            cmd,
            params={"interval": 0, "send": 0, "record": 0, "edge_send": 0, "edge_record": 0},
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.debug(cmd.pformat(response))
        time.sleep(1)

        # -- Set/Get I2C Setting --
        cmd = tsndcmd.SetI2CSetting()
        response, num_tot, num_ok = self.process_command(
            cmd,
            params={"interval": 0, "send": 0, "record": 0},
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.debug(cmd.pformat(response))
        time.sleep(1)

        # -- Set Quaternion Recoring Setting --
        cmd = tsndcmd.SetQuaternionSetting()
        response, num_tot, num_ok = self.process_command(
            cmd,
            params={"interval": 30, "send": 30, "record": 1},
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.debug(cmd.pformat(response))
        time.sleep(1)

        # -- Set/Get Ext 16bit AD Recoring Setting --
        cmd = tsndcmd.SetExt16ADSetting()
        response, num_tot, num_ok = self.process_command(
            cmd,
            params={"interval": 0, "send": 0, "record": 0, "mode_ch1": 0, "mode_ch2": 0, "mode_ch3": 0, "mode_ch4": 0},
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.debug(cmd.pformat(response))
        time.sleep(1)

        # == Set Acc Range ==
        cmd = tsndcmd.SetAccRange()
        response, num_tot, num_ok = self.process_command(
            cmd, params={"mode": 1},
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.debug(cmd.pformat(response))
        time.sleep(1)

        # -- Set Auto-power-off Setting --
        cmd = tsndcmd.SetAutoPowerOffTime()
        response, num_tot, num_ok = self.process_command(
            cmd, params={"minutes": 0},
            num_tot=num_tot, num_ok=num_ok,   
        )
        self.logger.info(cmd.pformat(response))
        time.sleep(1)

        # == Summary ==
        summary = {
            "total": num_tot,
            "suceeded":  num_ok,
            "failed": num_tot - num_ok
        }
        self.logger.info("== Summary ==")
        self.logger.info(f"{summary}")

    def get_device_status(self):        
        self.logger.debug("== Get Deivice Status ==")
        num_tot, num_ok = 0, 0
        
        # == Get Device Time ==
        cmd = tsndcmd.GetDeviceTime()
        response, num_tot, num_ok = self.process_command(
            cmd, num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.info(cmd.pformat(response))
        time.sleep(1)

        # == Get Ags Method ==
        cmd = tsndcmd.GetAgsMethod()
        response, num_tot, num_ok = self.process_command(
            cmd, num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.info(cmd.pformat(response))
        time.sleep(1)

        # == Get GeoMagnetic Method ==
        cmd = tsndcmd.GetGeoMagneticMethod()
        response, num_tot, num_ok = self.process_command(
            cmd, num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.info(cmd.pformat(response))
        time.sleep(1)

        # == Get Pressure Method ==
        cmd = tsndcmd.GetPresMethod()
        response, num_tot, num_ok = self.process_command(
            cmd,
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.info(cmd.pformat(response))
        time.sleep(1)

        # == Get Battery Method ==
        cmd = tsndcmd.GetBattMethod()
        response, num_tot, num_ok = self.process_command(
            cmd,
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.info(cmd.pformat(response))
        time.sleep(1)
        
        # == Get ExtInput Setting ==
        cmd = tsndcmd.GetExtInputSetting()
        response, num_tot, num_ok = self.process_command(
            cmd,
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.info(cmd.pformat(response))
        time.sleep(1)

        # == Get I2C Setting ==
        cmd = tsndcmd.GetI2CSetting()
        response, num_tot, num_ok = self.process_command(
            cmd,
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.info(cmd.pformat(response))
        time.sleep(1)

        # == Get Quaternion Recoring Setting ==
        cmd = tsndcmd.GetQuaternionSetting()
        response, num_tot, num_ok = self.process_command(
            cmd,
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.info(cmd.pformat(response))
        time.sleep(1)

        # == Get Ext 16bit AD Recoring Setting ==
        cmd = tsndcmd.GetExt16ADSetting()
        response, num_tot, num_ok = self.process_command(
            cmd,
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.info(cmd.pformat(response))
        time.sleep(1)

        # == Get Acc Range ==
        cmd = tsndcmd.GetAccRange()
        response, num_tot, num_ok = self.process_command(
            cmd, num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.info(cmd.pformat(response))
        time.sleep(1)

        # == Get Batt Status ==
        cmd = tsndcmd.GetBattStatus()
        response, num_tot, num_ok = self.process_command(
            cmd, num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.info(cmd.pformat(response))
        time.sleep(1)
        
        # == Get Device Status ==
        cmd = tsndcmd.GetDeviceStatus()
        response, num_tot, num_ok = self.process_command(
            cmd, num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.info(cmd.pformat(response))
        time.sleep(1)
        
        # == Get Auto-power-off Setting ==
        cmd = tsndcmd.GetAutoPowerOffTime()
        response, num_tot, num_ok = self.process_command(
            cmd,
            num_tot=num_tot, num_ok=num_ok,
        )
        self.logger.info(cmd.pformat(response))
        time.sleep(1)

        # == Summary ==
        summary = {
            "total": num_tot,
            "suceeded":  num_ok,
            "failed": num_tot - num_ok
        }
        self.logger.info("== Summary ==")
        self.logger.info(f"{summary}")


    def start_recording(self):
        self.logger.info("== Start Recoding ==")
        
        # == Start Recording ==
        cmd = tsndcmd.StartRecording()
        response, _, _ = self.process_command(cmd)
        self.logger.info(cmd.pformat(response))

    def stop_recording(self):
        self.logger.info("== Stop Recoding ==")
        
        # == Stop Recordiang ==
        cmd = tsndcmd.StopRecording()
        msg = cmd.encode()
        self.send(msg)       
        self.logger.info(f"Stop Recoding ({msg})")

    def check_memory_status(self):
        self.logger.info("== Check Memory Status ==")
        outputs = dict()
        
        # == Memory Counts ==
        cmd = tsndcmd.GetMemEntryCount()
        response, _, _ = self.process_command(cmd)
        self.logger.info(cmd.pformat(response))
        outputs.update({
            "MemEntryCount": response,
        })

        # == Free Memory Size ==
        cmd = tsndcmd.GetFreeMemSize()
        response, _, _ = self.process_command(cmd)
        self.logger.info(cmd.pformat(response))
        outputs.update({
            "FreeMemorySize": response,
        })
        return outputs

    def clear_memory(self):
        # == Memory Counts ==
        self.check_memory_status()

        self.logger.info("== Check Memory Status ==")
        print("Do you really want to clear memory? [Y/n] >>")
        res = input()
        if res == "Y":
            # == Clear Memory ==
            cmd = tsndcmd.ClearMemoery()
            response, _, _ = self.process_command(cmd)
            self.logger.info(cmd.pformat(response))

            # == Memory Counts ==
            cmd = tsndcmd.GetMemEntryCount()
            response = self.process_command(cmd)
            self.logger.info(cmd.pformat(response))
        else:
            self.logger.info("Quit")

    def check_entry_details(self):
        self.logger.info("== Check Entry Info ==")
        
        # == Check Entry Count ==
        outputs = self.check_memory_status()
        time.sleep(5)
        num_entry = outputs["MemEntryCount"]["num_entry"]
        
        # -- Download --
        # client.start_recording()
        for i in range(num_entry):
            cmd = tsndcmd.GetEntryInfo()
            response, _, _ = self.process_command(cmd, params={"entry_index": i+1})
            response["entry_index"] = i+1
            self.logger.info(cmd.pformat(response))
            time.sleep(1)

            cmd = tsndcmd.GetEntryDetail()
            response, _, _ = self.process_command(cmd, params={"entry_index": i+1})
            response["entry_index"] = i+1
            self.logger.info(cmd.pformat(response))
            time.sleep(1)


    def read_mem_data(self, entry_index):
        self.logger.info("== Read Mem Data ==")
        
        # -- Donwload meta-data --
        data = {"mode": "start", "entry": entry_index, "type": "meta-data"}
        self.logger.info(f"ReadMemDataCtl:: {data}")

        # Entry Info.
        cmd = tsndcmd.GetEntryInfo()
        self.logger.info(f"== Meta Data [Entry. {entry_index}] ==")
        response, _, _ = self.process_command(cmd, params={"entry_index": entry_index})
        response["entry_index"] = entry_index
        self.logger.info(cmd.pformat(response))
        time.sleep(1)

        # Entry Detail Info.
        cmd = tsndcmd.GetEntryDetail()
        response, _, _ = self.process_command(cmd, params={"entry_index": entry_index})
        response["entry_index"] = entry_index
        self.logger.info(cmd.pformat(response))
        time.sleep(1)

        # Start Read Memoery Data
        self.logger.info(f"== Body [Entry. {entry_index}] ==")
        cmd = tsndcmd.ReadMemData()
        msg = cmd.encode(entry_index)
        data = {"mode": "start", "entry": entry_index, "type": "body"}
        self.logger.info(f"ReadMemDataCtl:: {data}")
        self.send(msg)

        # Read and Output to Log File
        try:
            self.is_running = True
            self.event_listener()
            self.is_running = False
        except KeyboardInterrupt:
            self.is_running = False

            # Send StopReadMemData Command
            cmd = tsndcmd.StopReadMemData()
            msg = cmd.encode()
            data = {"mode": "stop", "entry": entry_index}
            self.logger.info(f"ReadMemDataCtl:: {data}")
            self.send(msg)

        # Finish Read Out the Entry
        data = {"mode": "end", "entry": entry_index}
        self.logger.info(f"ReadMemDataCtl:: {data}")
        