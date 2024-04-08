import argparse
import os
import yaml
from datetime import datetime, timezone, timedelta
from matplotlib import pyplot
from matplotlib.dates import DateFormatter
from matplotlib.backends.backend_pdf import PdfPages
from yamcs.client import YamcsClient

DEFAULT_YAMCS_URL = 'localhost:8090'
DEFAULT_YAMCS_INSTANCE = 'scsat1'

x_data = {}
y_data = {}
err_cnt = {}
configs = {}
params = []
statuses = []
param_name = {}


def write_pdf():

    global x_data
    global y_data

    with open(configs['pdf-name'], "wb") as file, PdfPages(file) as pdf:
        for param in configs['parameters']:
            target = param['name']
            pyplot.title(f"{target} (err: {err_cnt[target]})")
            pyplot.plot(x_data[target], y_data[target])
            pyplot.gca().xaxis.set_major_formatter(DateFormatter("%H:%M",
                                                   tz='Asia/Tokyo'))
            pyplot.gcf().autofmt_xdate()

            if 'TEMP' not in target and 'RW_' not in target:
                y_min, y_max = pyplot.gca().get_ylim()
                pyplot.gca().set_ylim(0, y_max * 1.1)

            pdf.savefig()
            pyplot.close()


def write_csv():

    global x_data
    global y_data

    for param in configs['parameters']:
        target = param['name']
        csvdir = configs['csv-dir']
        csvname = target.split('/')[-1]
        csvname = csvname.lower()
        with open(f"{csvdir}/{csvname}.csv", 'w',  encoding="utf-8") as outf:
            for idx, x in enumerate(x_data[target]):
                outf.write(f"{x},{y_data[target][idx]}\n")


def read_yamcs_archive(search_min, yamcs_url, yamcs_instance):

    client = YamcsClient(yamcs_url)
    archive = client.get_archive(instance=yamcs_instance)

    now = datetime.now(tz=timezone.utc)
    if search_min < 0:
        start = None
    else:
        start = now - timedelta(minutes=search_min)

    try:
        stream = archive.stream_parameter_values(params, start=start, stop=now)
        for pdata in stream:
            for data in pdata:
                x_data[data.name].append(data.generation_time)
                y_data[data.name].append(data.eng_value)
    except Exception as e:
        print(e)
        return False

    if len(statuses) == 0:
        return True

    try:
        stream = archive.stream_parameter_values(statuses, start=start, stop=now)
        for pdata in stream:
            for data in pdata:
                if data.eng_value != 0:
                    err_cnt[param_name[data.name]] += 1
    except Exception as e:
        print(e)
        return False

    return True


def init(yaml_file):

    global x_data
    global y_data
    global configs

    with open(yaml_file, 'r') as file:
        configs = yaml.safe_load(file)

    if not os.path.exists(configs['csv-dir']):
        os.makedirs(configs['csv-dir'])

    for param in configs['parameters']:
        target = param['name']
        x_data[target] = []
        y_data[target] = []
        err_cnt[target] = 0
        params.append(target)
        if 'status' in param and param['status'] != "":
            status = param['status']
            statuses.append(status)
            param_name[status] = target


def main(args):

    for yaml_file in args.yaml:

        if not os.path.exists(yaml_file):
            print(f"{yaml_file} is not exist, so skip the parse")
            continue
        else:
            print(f"Start to parse according to {yaml_file}")

        init(yaml_file)
        if not read_yamcs_archive(args.min, args.url, args.instance):
            return
        write_csv()
        write_pdf()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="SC-Sat1 Yamcs telemetry parse tool")
    parser.add_argument("--yaml", type=str, required=True, nargs="+",
                        help="Target yaml files")
    parser.add_argument("--min", type=int, default=-1,
                        help="Target minutes from current time for searching")
    parser.add_argument("--url", type=str, default=DEFAULT_YAMCS_URL,
                        help=f"Yamcs URL and Port number (default: {DEFAULT_YAMCS_URL})")
    parser.add_argument("--instance", type=str, default=DEFAULT_YAMCS_INSTANCE,
                        help=f"Yamcs instance name (default: {DEFAULT_YAMCS_INSTANCE})")
    args = parser.parse_args()
    main(args)
