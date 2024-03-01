import os
import sys
import re
from matplotlib import pyplot
from matplotlib.dates import DateFormatter
from matplotlib.backends.backend_pdf import PdfPages

from datetime import datetime

PDF_NAME = './main.pdf'
CSV_DIR = "./main_csv"

TARGET_LIST = [
    "OBC 1 Temperature:[deg]",
    "OBC 2 Temperature:[deg]",
    "OBC 3 Temperature:[deg]",
    "OBC XADC Temperature:[deg]",
    "I/O Board 1 Temperature:[deg]",
    "I/O Board 2 Temperature:[deg]",
    "X+ Temperature:[deg]",
    "X- Temperature:[deg]",
    "Y+ Temperature:[deg]",
    "Y- Temperature:[deg]",
    "Sun Sensor Y+ Temperature:[deg]",
    "Sun Sensor Y- Temperature:[deg]",
    "Magnetometer X+ Temperature:[deg]",
    "Magnetometer X- Temperature:[deg]",
    "Payload Board Temperature:[deg]",
    "OBC_1V0 Shunt:[mA]",
    "OBC_1V0 Bus:[v]",
    "OBC_1V8 Shunt:[mA]",
    "OBC_1V8 Bus:[v]",
    "OBC_3V3 Shunt:[mA]",
    "OBC_3V3 Bus:[v]",
    "OBC_3V3_SYSA Shunt:[mA]",
    "OBC_3V3_SYSA Bus:[v]",
    "OBC_3V3_SYSB Shunt:[mA]",
    "OBC_3V3_SYSB Bus:[v]",
    "OBC_3V3_IO Shunt:[mA]",
    "OBC_3V3_IO Bus:[v]",
    "OBC_XADC VCCINT:[v]",
    "OBC_XADC VCCAUX:[v]",
    "OBC_XADC VCCBRAM:[v]",
    "IO_PDU_04_3V3 Shunt:[mA]",
    "IO_PDU_04_3V3 Bus:[v]",
    "IO_VDD_3V3_SYS Shunt:[mA]",
    "IO_VDD_3V3_SYS Bus:[v]",
    "IO_VDD_3V3 Shunt:[mA]",
    "Uptime of EPS:[s]",
    "Uptime of SRS-3:[s]",
    "Uptime of ADCS Board:[s]",
    "Uptime of Payload Board:[s]",
    "Ping to EPS:[ms]",
    "Ping to SRS-3:[ms]",
    "Ping to ADCS Board:[ms]",
    "Ping to Payload Board:[ms]",
    "Payload Board JPEG Count:[cnt]",
    "DSTRX-3 HK FREE_COUNTER     :[cnt]",
    "DSTRX-3 HK WDT_COUNTER      :[cnt]",
    "DSTRX-3 HK RSSI             :[dBm]",
    "DSTRX-3 HK RCV_FREQ         :[Hz]",
    "DSTRX-3 HK TEMPERATURE      :[deg]",
    "DSTRX-3 HK VOLTAGE          :[v]",
    "DSTRX-3 HK TX_PWR           :[dBm]",
    "DSTRX-3 HK CARRIER_LOCK     :[bool]",
    "DSTRX-3 HK SUB_CARRIER_LOCK :[bool]",
    "DSTRX-3 HK TX_PWR_SET       :[dBm]",
    "DSTRX-3 HK BIT_RATE_SET     :[bps]",
    "DSTRX-3 HK PROG_NO          :[num]",
    "DSTRX-3 HK CHK_SUM          :[num]",
]

x_data = {}
y_data = {}


def get_shunt_ohm(key):

    ohm = 0

    if key == "OBC_1V0 Shunt":
        ohm = 0.01
    elif key == "OBC_1V8 Shunt":
        ohm = 0.01
    elif key == "OBC_3V3 Shunt":
        ohm = 0.01
    elif key == "OBC_3V3_SYSA Shunt":
        ohm = 0.01
    elif key == "OBC_3V3_SYSB Shunt":
        ohm = 0.01
    elif key == "OBC_3V3_IO Shunt":
        ohm = 0.01
    elif key == "IO_PDU_04_3V3 Shunt":
        ohm = 0.1
    elif key == "IO_VDD_3V3_SYS Shunt":
        ohm = 0.1
    elif key == "IO_VDD_3V3 Shunt":
        ohm = 0.1

    return ohm


def extract(line, target, key):

    global x_data
    global y_data

    dat = line.split(']')
    date = dat[0].replace('[', '')
    dat = line.split(key)
    if len(dat) > 1:
        val = dat[-1].split(' ')
        if len(val) == 1 or val[1] == 'Failed':
            return
        x_data[target].append(datetime.strptime(date, "%Y-%m-%d %H:%M:%S"))
        if '.' in val[1]:
            y_data[target].append(float(val[1]))
        elif 'Shunt' in key:
            ohm = get_shunt_ohm(key)
            y_data[target].append(float(int(val[1])/1000/ohm))
        elif 'Bus' in key:
            y_data[target].append(float(int(val[1])/1000))
        else:
            y_data[target].append(int(val[1]))


def parse(line):

    for target in TARGET_LIST:
        key, unit = target.split(':')
        if f': {key}' in line:
            extract(line, target, key)
            break


def write_pdf():

    global x_data
    global y_data

    with open(PDF_NAME, "wb") as file, PdfPages(file) as pdf:
        for target in TARGET_LIST:
            pyplot.title(f"{target}")
            pyplot.plot(x_data[target], y_data[target])
            pyplot.gca().xaxis.set_major_formatter(DateFormatter("%H:%M"))
            pyplot.gcf().autofmt_xdate()

            if 'Tem' not in target:
                y_min, y_max = pyplot.gca().get_ylim()
                pyplot.gca().set_ylim(0, y_max * 1.1)

            pdf.savefig()
            pyplot.close()


def write_csv():

    global x_data
    global y_data

    for target in TARGET_LIST:
        csvname = re.sub(r'[/:]+', '', target.split(':')[0]).replace(' ', '_')
        csvname = csvname.lower()
        with open(f"{CSV_DIR}/{csvname}.csv", 'w',  encoding="utf-8") as outf:
            for idx, x in enumerate(x_data[target]):
                outf.write(f"{x},{y_data[target][idx]}\n")


def read_log_file(infile):

    with open(infile, 'r', encoding="utf-8") as inf:
        while True:
            line = inf.readline()
            if not line:
                break
            else:
                line = line.replace('\n', '')
                parse(line)


def init():

    global x_data
    global y_data

    if not os.path.exists(CSV_DIR):
        os.makedirs(CSV_DIR)

    for target in TARGET_LIST:
        x_data[target] = []
        y_data[target] = []


def main(infile):

    init()
    read_log_file(infile)
    write_csv()
    write_pdf()


if __name__ == '__main__':
    args = sys.argv
    main(args[1])
