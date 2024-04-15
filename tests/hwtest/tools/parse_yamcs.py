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
BLANK_TIME_SEC = 60

x_data = {}
y_data = {}
err_cnt = {}
configs = {}
params = []
statuses = []
param_name = {}
prev_time = {}


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

            if 'TEMP' not in target and 'temp' not in target and 'RW_' not in target:
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


def read_yamcs_archive(args):

    search_min = args.min
    yamcs_url = args.url
    yamcs_instance = args.instance
    start_time = args.start
    end_time = args.end

    client = YamcsClient(yamcs_url)
    archive = client.get_archive(instance=yamcs_instance)

    now = datetime.now(tz=timezone.utc)

    if end_time is None:
        stop = now
    else:
        stop = datetime.strptime(end_time + '+0900', '%Y-%m-%d %H:%M:%S%z')

    if search_min < 0:
        if start_time is not None:
            start = datetime.strptime(start_time + '+0900', '%Y-%m-%d %H:%M:%S%z')
        else:
            start = None
    else:
        start = stop - timedelta(minutes=search_min)

    try:
        stream = archive.stream_parameter_values(params, start=start, stop=stop)
        for pdata in stream:
            for data in pdata:
                target = data.name

                # Insert NaN value when detect the blank time period
                if not prev_time[target] is None:
                    diff = data.generation_time - prev_time[target]
                    if diff.total_seconds() > BLANK_TIME_SEC:
                        x_data[target].append(data.generation_time -
                                              timedelta(seconds=BLANK_TIME_SEC))
                        y_data[target].append(None)

                x_data[target].append(data.generation_time)
                y_data[target].append(data.eng_value)
                prev_time[target] = data.generation_time
    except Exception as e:
        print(e)
        return False

    if len(statuses) == 0:
        return True

    try:
        stream = archive.stream_parameter_values(statuses, start=start, stop=stop)
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
        prev_time[target] = None
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
        if not read_yamcs_archive(args):
            return
        write_csv()
        write_pdf()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="SC-Sat1 Yamcs telemetry parse tool")
    parser.add_argument("--yaml", type=str, required=True, nargs="+",
                        help="Target yaml files")
    parser.add_argument("--url", type=str, default=DEFAULT_YAMCS_URL,
                        help=f"Yamcs URL and Port number (default: {DEFAULT_YAMCS_URL})")
    parser.add_argument("--instance", type=str, default=DEFAULT_YAMCS_INSTANCE,
                        help=f"Yamcs instance name (default: {DEFAULT_YAMCS_INSTANCE})")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--min", type=int, default=-1,
                       help="Target minutes from current time for searching")
    group.add_argument("--start", type=str, default=None,
                       help="Start time for searching")
    parser.add_argument("--end", type=str, default=None,
                        help="End time for searching")
    args = parser.parse_args()
    main(args)
