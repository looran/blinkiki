#!/usr/bin/env python3

DESCRIPTION = "d"
EXAMPLES = """examples:
"""

import sys
import csv
import asyncio
import argparse
from datetime import datetime, timedelta
from pathlib import Path
from struct import unpack
from contextlib import asynccontextmanager

from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
import uvicorn

import pandas as pd
import panel as pn
import holoviews as hv
import hvplot.pandas
from panel.io.fastapi import add_application
from bokeh.sampledata.penguins import data as df
from holoviews.plotting.links import RangeToolLink
from holoviews.streams import Buffer
from starlette.websockets import WebSocketDisconnect

CSV_SAVE_INTERVAL = 60
CSV_PATH = Path("blinkiki.csv")

class Store:
    FIELDS = ["date", "pulses", "reports", "err"]
    def __init__(self, path):
        self.dat = list()
        self.path = path

    def load(self):
        if self.path.exists():
            sys.stderr.write(f"loading {self.path}...")
            with self.path.open(newline='') as fd:
                for entry in csv.DictReader(fd):
                    entry["date"] = datetime.fromisoformat(entry["date"])
                    entry["reports"] = int(entry["reports"])
                    entry["err"] = int(entry["err"])
                    self.dat.append(entry)
            sys.stderr.write(f"done\n")

    def save(self):
        sys.stderr.write(f"saving {self.path}...")
        with self.path.open('w', newline='') as fd:
            writer = csv.DictWriter(fd, self.FIELDS)
            writer.writeheader()
            for rec in self.dat:
                writer.writerow(rec)
        sys.stderr.write(f"done\n")

    def report(self, data, sens_time):
        now = datetime.now().replace(microsecond=0)
        for num, pulses in reversed(list(enumerate(data))):
            print(f"report {num}: {pulses}")
            if pulses == 255:
                continue
            date = now - timedelta(seconds=5) * num
            rec = self.find(date)
            if not rec:
                rec = dict.fromkeys(self.FIELDS, 0)
                rec["date"] = date
                rec["pulses"] = pulses
                self.dat.append(rec)
            if pulses != rec["pulses"]:
                rec["err"] += 1
            rec["reports"] += 1

    def find(self, date):
        print(f"    find {date}")
        for rec in reversed(self.dat):
            if ( (date >= rec["date"] and date - rec["date"] < timedelta(seconds=2))
                    or (date <= rec["date"] and rec["date"] - date < timedelta(seconds=2)) ):
                print(f"       diff {rec['date']} match")
                return rec
            if rec["date"] < date - timedelta(seconds=5):
                print(f"       diff {rec['date']} exit")
                return False
            print(f"       diff {rec['date']} loop")
        return False


@asynccontextmanager
async def app_lifespan(app: FastAPI):
    async def _bg_store_save(app: FastAPI):
        while True:
            await asyncio.sleep(CSV_SAVE_INTERVAL)
            app._store.save()
    if app._store:
        app._store.load()
        Path("up.log").open("a").write(f"{datetime.now()} server start\n")
        asyncio.create_task(_bg_store_save(app))
    yield
    if app._store:
        app._store.save()

app = FastAPI(lifespan=app_lifespan)

@app.get("/", response_class=HTMLResponse)
async def home():
    return "<h1>welcome</h1>"

@add_application('/stats', app=app, title='blinkiki stats')
def stats():
    df = pd.read_csv("blinkiki_8087.csv", date_format="%Y-%m-%d %H:%M:%S", index_col="date")
    pn.config.sizing_mode = "stretch_width"

    # bounds for heures creuses - heures pleines
    start_from = df["pulses"].index[0].replace(hour=22, minute=30, second=0) - timedelta(days=1)
    start_to = df["pulses"].index[-1].replace(hour=22, minute=30, second=0)
    x_start = pd.date_range(start_from, start_to, freq='1D')
    end_from = start_from.replace(hour=6, minute=30, second=0) + timedelta(days=1)
    end_to = start_to.replace(hour=6, minute=30, second=0) + timedelta(days=1)
    x_end = pd.date_range(end_from, end_to, freq='1D')
    hchp = hv.VSpans((x_start, x_end)).opts(color='#EEF', xlim=(df["pulses"].index[0], df["pulses"].index[-1]))

    # main graph
    df["cumsum"] = df["pulses"].cumsum()
    mean = df["pulses"].resample('min').mean()
    main_mean = hv.Curve(mean, vdims=['pulses'], label="power consumption measured").opts(
            ylabel='Wh / min measured', height=800,
            tools=['xwheel_zoom', 'xpan'],
            active_tools=['xwheel_zoom', 'xpan'],
            default_tools= ['reset', 'xwheel_zoom', 'xpan', 'save', 'hover'])
    main_cumsum = hv.Curve(df["cumsum"], vdims=['cumsum'], label="power consumption total").opts(
            autorange='y', ylabel='Wh total', fontsize={'ylabel':10}, color="#FCC")
    main = (main_mean * main_cumsum * hchp).opts(multi_y=True, responsive=True, legend_position='top_left')

    # overview graph
    overview = hchp * hv.Curve(mean, vdims=['pulses']).opts(
            ylabel='Wh / min', height=125, axiswise=True,
            default_tools=['reset', 'xwheel_zoom', 'xpan'],
            tools=['xwheel_zoom', 'xpan'],
            active_tools=['xwheel_zoom', 'xpan'], responsive=True)

    # combine main and overview graphs
    RangeToolLink(overview, main)
    g_main = (main + overview).opts(merge_tools=True).cols(1)

    # mesures graph
    g_pulses = df.hvplot.scatter(y=["pulses", "err", "reports"]).opts(default_tools=['hover']) * hchp

    return pn.Column(
            pn.pane.HoloViews(g_main, sizing_mode="stretch_width"),
            pn.Card(pn.pane.HoloViews(g_pulses, height=400, sizing_mode="stretch_width"), title="measures", collapsed=True) )

@add_application('/stream_test', app=app, title='blinkiki stream')
def stream_test():
    async def _send(buffer, dmap):
        print("send start")
        for i in range(100):
            j = i*2
            dat = pd.DataFrame([(i, j)], columns=['x', 'y'])
            ret = buffer.send(dat)
            await asyncio.sleep(0.1)
        print("send end")

    example = pd.DataFrame({'x': [], 'y': []}, columns=['x', 'y'])
    buf = Buffer(example, length=100, index=False)
    dmap = hv.DynamicMap(hv.Curve, streams=[buf])
    asyncio.create_task(_send(buf, dmap))
    return pn.pane.HoloViews(dmap, height=600, sizing_mode="stretch_width")

@app.post("/sensor/report", response_class=HTMLResponse)
async def sensor_report(request: Request):
    if not request.app._store:
        return "readonly"
    data: bytes = await request.body()
    print(data)
    if len(data) < 5:
        return "invalid"
    sens_id, sens_ver, sens_type, sens_time = unpack('<bbbi', data[:7])
    request.app._store.report(data[7:], sens_time)
    return "ok"

@app.post("/sensor/up", response_class=HTMLResponse)
async def sensor_up(request: Request):
    if not request.app._store:
        return "readonly"
    Path("up.log").open("a").write(f"{datetime.now()} up\n")
    return "ok"

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=DESCRIPTION, epilog=EXAMPLES, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('-p', '--port', type=int, default=8087)
    parser.add_argument('-r', '--readonly', action="store_true", help="start a read-only instance, does not accept sensor reports")
    parser.add_argument('-v', '--verbose', action='store_true')

    args = parser.parse_args()

    app._store = None
    if not args.readonly:
        app._store = Store(CSV_PATH)
    uvicorn.run(app, host="0.0.0.0", port=args.port, loop="asyncio") # loop asyncio for debugging (compatibility with IPython embed)

""" some holoviews tests

    slider = pn.widgets.IntSlider(name='Slider', start=0, end=10, value=3)
    graph = df.hvplot.scatter(x='bill_length_mm', y='bill_depth_mm', by='species')
    from IPython import embed; import nest_asyncio; nest_asyncio.apply(); embed(using='asyncio')
    return pn.Row(
            pn.pane.HoloViews(graph, height=300, sizing_mode="stretch_width"),
            slider.rx() * '⭐'
            )
    graph = df.hvplot.line(y='pulses').opts(apply_hard_bounds=True, subcoordinate_y=True, subcoordinate_scale=1.2)
    graph = df.hvplot.line(y='pulses').opts(subcoordinate_y=True, subcoordinate_scale=1.2)
    graph = df.hvplot.line(y='pulses').opts(apply_hard_bounds=True, ylim=(0,6),normalize=True)
    graph = df.hvplot.line(y='pulses').opts(apply_hard_bounds=True)
    graph = graph.redim.range(x_col=(None, None))
    g_mean = mean.hvplot.line().opts(tools=['xwheel_zoom', 'xpan'], active_tools=['xwheel_zoom', 'xpan'], default_tools= ['reset', 'xwheel_zoom', 'xpan', 'save', 'hover'], ylim=(0,None))
    mean = df["pulses"].resample('min').mean()
     , ylim=(0,6)),

    df["cumsum"] = df["pulses"].cumsum()
    g_main = (
            hv.Curve(df["pulses"].resample('min').mean(), vdims=['pulses']).opts(
                ylabel='Wh', tools=['xwheel_zoom', 'xpan'], active_tools=['xwheel_zoom', 'xpan'], default_tools= ['reset', 'xwheel_zoom', 'xpan', 'save', 'hover'])
            * hv.Curve(df["cumsum"], vdims=['cumsum']).opts(
                autorange='y', ylabel='Wh total', fontsize={'ylabel':10}, color="#FCC")
    ).opts(multi_y=True)
"""
