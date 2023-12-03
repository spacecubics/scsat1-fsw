import os
import sys
import re
from matplotlib import pyplot
from matplotlib.dates import DateFormatter
from matplotlib.backends.backend_pdf import PdfPages
from datetime import datetime

PDF_NAME = './srs3.pdf'
CSV_DIR = "./srs3_csv"

TARGET_LIST = [
    "bootcount:[cnt]",
    "gwdt.counter:[cnt]",
    "temp.mcu:[deg]",
    "temp.power:[deg]",
    "temp.lna:[deg]",
    "temp.pa:[deg]",
    "volt.vin:[v]",
    "volt.vreg:[v]",
    "volt.3v3:[v]",
    "cur.vin:[mA]",
    "cur.vreg:[mA]",
    "cur.3v3:[mA]",
    "power.vin:[mW]",
    "power.vreg:[mW]",
    "power.3v3:[mW]",
]

x_data = {}
y_data = {}


def extract(line, target, key):

    global x_data
    global y_data

    dat = line.split(']')
    date = dat[0].replace('[', '')
    dat = line.split(key)
    if len(dat) > 1:
        val = dat[-1].split()
        if len(val) > 3:
            x_data[target].append(datetime.strptime(date, "%Y-%m-%d %H:%M:%S.%f"))
            if 'temp' in target:
                y_data[target].append(float(int(val[3])/100))
            elif 'volt' in target:
                y_data[target].append(float(int(val[3])/1000))
            else:
                y_data[target].append(int(val[3]))


def parse(line):

    for target in TARGET_LIST:
        key, unit = target.split(':')
        if f'{key}' in line:
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

            if 'temp' not in target:
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
                line = line.replace('\r\n', '')
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
