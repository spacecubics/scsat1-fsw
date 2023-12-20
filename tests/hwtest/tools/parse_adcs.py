import os
import sys
import re
from matplotlib import pyplot
from matplotlib.dates import DateFormatter
from matplotlib.backends.backend_pdf import PdfPages
from datetime import datetime

PDF_NAME = './adcs.pdf'
CSV_DIR = "./adcs_csv"

TARGET_LIST = [
    "OBC 1 Temperature:[deg]",
    "OBC 2 Temperature:[deg]",
    "OBC 3 Temperature:[deg]",
    "OBC XADC Temperature:[deg]",
    "ADCS Board 1 Temperature:[deg]",
    "ADCS Board 2 Temperature:[deg]",
    "ADCS RW Temperature:[deg]",
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
    "ADCS_VDD_3V3_IMU Shunt:[mA]",
    "ADCS_VDD_3V3_IMU Bus:[v]",
    "ADCS_VDD_3V3_GPS Shunt:[mA]",
    "ADCS_VDD_3V3_GPS Bus:[v]",
    "ADCS_VDD_3V3_DRV Shunt:[mA]",
    "ADCS_VDD_3V3_DRV Bus:[v]",
    "ADCS_VDD_12V_DRVX Shunt:[mA]",
    "ADCS_VDD_12V_DRVX Bus:[v]",
    "ADCS_VDD_12V_DRVY Shunt:[mA]",
    "ADCS_VDD_12V_DRVY Bus:[v]",
    "ADCS_VDD_12V_DRVZ Shunt:[mA]",
    "ADCS_VDD_12V_DRVZ Bus:[v]",
    "RW X count:[cnt]",
    "RW Y count:[cnt]",
    "RW Z count:[cnt]",
]

GNSS_TARGET_LIST = [
    "GNSS Temperature:[deg]",
    "GNSS Digital Core 3V3:[v]",
    "GNSS Antenna Voltage:[v]",
    "GNSS Digital 1V2 Core Voltage:[v]",
    "GNSS Regulated Supply Voltage:[v]",
    "GNSS 1V8:[v]",
]

GNSS_IDX_LIST = [
    10,
    12,
    14,
    16,
    18,
    20,
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
    elif key == "ADCS_VDD_3V3_IMU Shunt":
        ohm = 0.1
    elif key == "ADCS_VDD_3V3_GPS Shunt":
        ohm = 0.1
    elif key == "ADCS_VDD_3V3_DRV Shunt":
        ohm = 0.1
    elif key == "ADCS_VDD_12V_DRVX Shunt":
        ohm = 1
    elif key == "ADCS_VDD_12V_DRVY Shunt":
        ohm = 1
    elif key == "ADCS_VDD_12V_DRVZ Shunt":
        ohm = 1

    return ohm


def extract_gnss(line, target, idx):

    global x_data
    global y_data

    dat = line.split(']')
    date = dat[0].replace('[', '')
    dat = line.split(',')
    if len(dat) >= idx:
        x_data[target].append(datetime.strptime(date, "%Y-%m-%d %H:%M:%S"))
        y_data[target].append(float(dat[idx]))


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
        if 'Shunt' in key:
            ohm = get_shunt_ohm(key)
            if '.' in val[1]:
                y_data[target].append(float(float(val[1])/1000/ohm))
            else:
                y_data[target].append(float(int(val[1])/1000/ohm))
        elif 'Bus' in key:
            if '.' in val[1]:
                y_data[target].append(float(float(val[1])/1000))
            else:
                y_data[target].append(float(int(val[1])/1000))
        elif '.' in val[1]:
            y_data[target].append(float(val[1]))
        else:
            y_data[target].append(int(val[1]))


def parse(line):

    for target in TARGET_LIST:
        key, unit = target.split(':')
        if f': {key}' in line:
            extract(line, target, key)
            break

    if "#HWMONITORA" in line:
        for idx, target in enumerate(GNSS_TARGET_LIST):
            extract_gnss(line, target, GNSS_IDX_LIST[idx])


def write_pdf():

    global x_data
    global y_data

    with open(PDF_NAME, "wb") as file, PdfPages(file) as pdf:
        for target in TARGET_LIST + GNSS_TARGET_LIST:
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

    for target in TARGET_LIST + GNSS_TARGET_LIST:
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

    for target in TARGET_LIST + GNSS_TARGET_LIST:
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
