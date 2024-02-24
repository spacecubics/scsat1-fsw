import os
import sys
import re
import yaml
from matplotlib import pyplot
from matplotlib.dates import DateFormatter
from matplotlib.backends.backend_pdf import PdfPages
from yamcs.client import YamcsClient

from datetime import datetime

DEFAULT_YAMCS_URL='localhost:8090'
YAMCS_INSTANCE='scsat1'
PDF_NAME = './main.pdf'
CSV_DIR = "./main_csv"

x_data = {}
y_data = {}
err_cnt = {}
configs = {}


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


def write_pdf():

    global x_data
    global y_data

    with open(PDF_NAME, "wb") as file, PdfPages(file) as pdf:
        for param in configs['parameters']:
            target = param['name']
            pyplot.title(f"{target} (err: {err_cnt[target]})")
            pyplot.plot(x_data[target], y_data[target])
            pyplot.gca().xaxis.set_major_formatter(DateFormatter("%H:%M"))
            pyplot.gcf().autofmt_xdate()

            if 'TEMP' not in target:
                y_min, y_max = pyplot.gca().get_ylim()
                pyplot.gca().set_ylim(0, y_max * 1.1)

            pdf.savefig()
            pyplot.close()


def write_csv():

    global x_data
    global y_data

    for param in configs['parameters']:
        target = param['name']
        csvname = re.sub(r'[/:]+', '', target.split(':')[0]).replace(' ', '_')
        csvname = csvname.lower()
        with open(f"{CSV_DIR}/{csvname}.csv", 'w',  encoding="utf-8") as outf:
            for idx, x in enumerate(x_data[target]):
                outf.write(f"{x},{y_data[target][idx]}\n")


def read_yamcs_archive():

    client = YamcsClient(configs['yamcs-url'])
    archive = client.get_archive(instance=YAMCS_INSTANCE)

    for param in configs['parameters']:
        target = param['name']
        stream = archive.stream_parameter_values(target);
        for pdata in stream:
            for data in pdata:
                x_data[target].append(data.generation_time)
                y_data[target].append(data.raw_value)

        stream = archive.stream_parameter_values(param['status']);
        for pdata in stream:
            for data in pdata:
                if data.raw_value != 0:
                    err_cnt[target] += 1


def init(yaml_file):

    global x_data
    global y_data
    global configs

    if not os.path.exists(CSV_DIR):
        os.makedirs(CSV_DIR)

    with open(yaml_file, 'r') as file:
        configs = yaml.safe_load(file)

    for param in configs['parameters']:
        target = param['name']
        x_data[target] = []
        y_data[target] = []
        err_cnt[target] = 0


def main(yaml_file):

    init(yaml_file)
    read_yamcs_archive()
    write_csv()
    write_pdf()


if __name__ == '__main__':
    args = sys.argv
    main(args[1])
