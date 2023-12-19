from pathlib import Path
import numpy as np
import pandas as pd
import json

from logging import getLogger
logger = getLogger("tsndctl.postprocessing")

METADATA_KEYS = [
    "MemEntryCount",
    "FreeMemSize",
    "ReadMemDataCtl", 
    "EntryInfo",
    "EntryDetail",
]


def summarize_log_parse_results(metadata, df_ags, messages):
    df_ags = df_ags[df_ags["ts"] > 0].reset_index(drop=True)
    warnings = [msg for msg in messages if "WARNING" in msg]

    num_records = metadata.get("EntryInfo")[0].get("num_records")
    error_rate = 1. - len(df_ags) / (num_records / 2)
    
    if len(df_ags) > 0:
        ts = df_ags["ts"].values
        duration = (ts[-1] - ts[0]) / 1000.
    else:
        duration = 0

    summary = {
        "records": num_records,
        "ags_expected": num_records//2,
        "ags": len(df_ags),
        "messages": len(messages),
        "warnings": len(warnings),
        "error_rates": error_rate,
        "duration": duration,
    }
    return summary 

def parse_json(data):
    data = data.strip().replace("\n", "").replace("'", "\"")
    return json.loads(data)


def parse_logfile(logfile):
    """
    Args:
        logfile (str): path to a target logfile.
    """
    metadata = dict()
    df_ags = []
    messages = []
    with open(logfile, "r") as f:
        lines = f.readlines()
        for i, line in enumerate(lines[:]):
            line = line.replace("\n", "")
            if "::" not in line:
                messages.append(line)
                continue

            line = line.split(" - ")
            if len(line) == 2:
                _, body = line
                if len(body.split("::")) == 2:
                    body_key, body_dict = body.split("::")
                    body_dict = parse_json(body_dict)
                else:
                    body_key = body
                    body_dict = dict()
                
                if body_key in METADATA_KEYS:
                    if body_key not in metadata.keys():
                        metadata[body_key] = []
                    metadata[body_key].append(body_dict)                    

                elif body_key == "AgsDataEvent":
                    df_ags.append({
                        "ts": body_dict.get("ts"),
                        "acc_x": body_dict.get("acc")[0],
                        "acc_y": body_dict.get("acc")[1],
                        "acc_z": body_dict.get("acc")[2],
                        "gyro_x": body_dict.get("gyro")[0],
                        "gyro_y": body_dict.get("gyro")[1],
                        "gyro_z": body_dict.get("gyro")[2],
                    })
            else:
                logger.warning(f"unknown record: {line}")
        
        df_ags = pd.DataFrame(df_ags)
        summary = summarize_log_parse_results(metadata, df_ags, messages)
        return (metadata, df_ags, messages), summary


