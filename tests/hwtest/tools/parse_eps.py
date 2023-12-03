import os
import sys
import re
from matplotlib import pyplot
from matplotlib.dates import DateFormatter
from matplotlib.backends.backend_pdf import PdfPages
from datetime import datetime

PDF_NAME = "./eps.pdf"
CSV_DIR = "./eps_csv"

TARGET_LIST = [
    "RTC Time",
    "BATTERY Voltage",
    "OUTPUT CONVERTERS 1",
    "OUTPUT CONVERTERS 2",
    "OUTPUT CONVERTERS 3",
    "OUTPUT CONVERTERS 4",
    "TEMPERATURE MPPT CONVERTERS 1",
    "TEMPERATURE MPPT CONVERTERS 2",
    "TEMPERATURE MPPT CONVERTERS 3",
    "TEMPERATURE MPPT CONVERTERS 4",
    "TEMPERATURE OUTPUT CONVERTERS 5",
    "TEMPERATURE OUTPUT CONVERTERS 6",
    "TEMPERATURE OUTPUT CONVERTERS 7",
    "TEMPERATURE OUTPUT CONVERTERS 8",
    "TEMPERATURE BATTERY PACK 9",
    "TEMPERATURE BATTERY PACK 12",
    "WDT_gs COUNTER Count",
    "WDT_gs COUNTER Left",
    "BOOT COUNTER Counter",
]

KEY_LIST = [
    "curr rtc time",
    "Voltage (mV)",
    "Converter      1      2      3      4 ",
    "Converter      1      2      3      4 ",
    "Converter      1      2      3      4 ",
    "Converter      1      2      3      4 ",
    " 1: ",
    " 2: ",
    " 3: ",
    " 4: ",
    " 5: ",
    " 6: ",
    " 7: ",
    " 8: ",
    " 9: ",
    " 12: ",
    "Count:",
    "Left:",
    "Counter:",
]


def extract_eps(line, target, idx, x_data, y_data, outfile):

    dat = line.split(']')
    date = dat[0].replace('[', '')
    dat = line.split()
    unit = ''

    if target.startswith('RTC Time'):
        val = float(dat[idx])
        unit = 's'
    elif target.startswith('BATTERY Voltage'):
        val = float(int(dat[idx])/1000)
        unit = 'v'
    elif target.startswith('OUTPUT CONVERTERS'):
        val = float(int(dat[idx])/1000)
        unit = 'v'
    elif target.startswith('TEMPERATURE MPPT CONVERTERS'):
        val = int(dat[idx])
        unit = 'deg'
    elif target.startswith('TEMPERATURE OUTPUT CONVERTERS'):
        val = int(dat[idx])
        unit = 'deg'
    elif target.startswith('TEMPERATURE BATTERY PACK'):
        val = int(dat[idx])
        unit = 'deg'
    elif target.startswith('WDT_gs COUNTER Count'):
        val = int(dat[idx])
        unit = 'cnt'
    elif target.startswith('WDT_gs COUNTER Left'):
        val = int(dat[idx])
        unit = 's'
    elif target.startswith('BOOT COUNTER Counter'):
        val = int(dat[idx])
        unit = 'cnt'

    outfile.write(f"{date},{val}\n")
    x_data.append(datetime.strptime(date, "%Y-%m-%d %H:%M:%S.%f"))
    y_data.append(val)

    return unit


def parse_eps(infile, target, key, idx, pdf):

    x_data = []
    y_data = []
    unit = ''

    csvname = target.replace(' ', '_').lower()
    with open(f"{CSV_DIR}/{csvname}.csv", 'w',  encoding="utf-8") as outf:
        with open(infile, 'r', encoding="utf-8") as inf:
            eps_entry_count = -1
            while True:
                line = inf.readline()
                if not line:
                    break
                else:
                    line = line.replace('\n', '')
                    if key in line:
                        eps_entry_count = 0
                    elif eps_entry_count >= 0:
                        eps_entry_count += 1

                    ret = ''
                    if target.startswith('RTC Time') and eps_entry_count == 0:
                        ret = extract_eps(line, target, idx, x_data, y_data, outf)
                    elif target.startswith('BATTERY Voltage') and eps_entry_count == 0:
                        ret = extract_eps(line, target, idx, x_data, y_data, outf)
                    elif target.startswith('OUTPUT CONVERTERS') and eps_entry_count == 1:
                        ret = extract_eps(line, target, idx, x_data, y_data, outf)
                    elif target.startswith('TEMPERATURE MPPT CONVERTERS') and eps_entry_count == 0:
                        ret = extract_eps(line, target, idx, x_data, y_data, outf)
                    elif target.startswith('TEMPERATURE OUTPUT CONVERTERS') and eps_entry_count == 0:
                        ret = extract_eps(line, target, idx, x_data, y_data, outf)
                    elif target.startswith('TEMPERATURE BATTERY PACK') and eps_entry_count == 0:
                        ret = extract_eps(line, target, idx, x_data, y_data, outf)
                    elif target.startswith('WDT_gs COUNTER Count') and eps_entry_count == 0:
                        ret = extract_eps(line, target, idx, x_data, y_data, outf)
                    elif target.startswith('WDT_gs COUNTER Left') and eps_entry_count == 0:
                        ret = extract_eps(line, target, idx, x_data, y_data, outf)
                    elif target.startswith('BOOT COUNTER Counter') and eps_entry_count == 0:
                        ret = extract_eps(line, target, idx, x_data, y_data, outf)

                    if ret:
                        unit = ret

    pyplot.title(f"{target}:[{unit}]")
    pyplot.plot(x_data, y_data)
    pyplot.gca().xaxis.set_major_formatter(DateFormatter("%H:%M"))
    pyplot.gcf().autofmt_xdate()
    pdf.savefig()
    pyplot.close()


def parse(infile):

    with open(PDF_NAME, "wb") as file, PdfPages(file) as pdf:

        for idx, target in enumerate(TARGET_LIST):
            if target.startswith('RTC Time'):
                parse_eps(infile, target, KEY_LIST[idx], 5, pdf)
            elif target.startswith('BATTERY Voltage'):
                parse_eps(infile, target, KEY_LIST[idx], 4, pdf)
            elif target.startswith('OUTPUT CONVERTERS 1'):
                parse_eps(infile, target, KEY_LIST[idx], 4, pdf)
            elif target.startswith('OUTPUT CONVERTERS 2'):
                parse_eps(infile, target, KEY_LIST[idx], 5, pdf)
            elif target.startswith('OUTPUT CONVERTERS 3'):
                parse_eps(infile, target, KEY_LIST[idx], 6, pdf)
            elif target.startswith('OUTPUT CONVERTERS 4'):
                parse_eps(infile, target, KEY_LIST[idx], 7, pdf)
            elif target.startswith('TEMPERATURE MPPT CONVERTERS'):
                parse_eps(infile, target, KEY_LIST[idx], 3, pdf)
            elif target.startswith('TEMPERATURE OUTPUT CONVERTERS'):
                parse_eps(infile, target, KEY_LIST[idx], 3, pdf)
            elif target.startswith('TEMPERATURE BATTERY PACK'):
                parse_eps(infile, target, KEY_LIST[idx], 3, pdf)
            elif target.startswith('WDT_gs COUNTER Count'):
                parse_eps(infile, target, KEY_LIST[idx], 3, pdf)
            elif target.startswith('WDT_gs COUNTER Left'):
                parse_eps(infile, target, KEY_LIST[idx], 3, pdf)
            elif target.startswith('BOOT COUNTER Counter'):
                parse_eps(infile, target, KEY_LIST[idx], 3, pdf)


def init(infile):

    if not os.path.exists(CSV_DIR):
        os.makedirs(CSV_DIR)


def main(infile):

    init(infile)
    parse(infile)


if __name__ == '__main__':
    args = sys.argv
    main(args[1])
