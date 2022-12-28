from flask import Flask, render_template
from flask_mqtt import Mqtt
from flask_socketio import SocketIO
from flask_bootstrap import Bootstrap
from flask_cors import CORS, cross_origin

import pandas as pd
import json
import plotly
import plotly.express as px

from datetime import datetime

app = Flask(__name__)
app.config['MQTT_BROKER_URL'] = "broker.hivemq.com"
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = "maker:41gccdzr2xfld2hOClZuOvtiYzz6fJYcaWQOniFw"
app.config['MQTT_PASSWORD'] = 'xxxxxx'
app.config['MQTT_REFRESH_TIME'] = 1.0  # refresh time in seconds
app.config['MQTT_KEEPALIVE'] = 10  # set the time interval for sending a ping to the broker to 5 seconds
app.config['ATT_DEVICE_ID'] = "iQVwKTiMb0icOtYTpi8Jsgkp"

mqtt = Mqtt(app)
socketio = SocketIO(app,cors_allowed_origins='*', async_mode="eventlet")

@mqtt.on_connect()
def handle_connect(client, userdata, flags, rc):
    #mqtt.subscribe('device/iQVwKTiMb0icOtYTpi8Jsgkp/asset/temperature/feed')
    mqtt.subscribe('iot_jethro')

@mqtt.on_message()
def handle_mqtt_message(client, userdata, message):
    data = dict(
        topic=message.topic,
        payload=message.payload.decode()
    )
    dt = datetime.now()
    str_date_time = dt.strftime("%d-%m-%Y, %H:%M:%S")
    
    df = pd.read_csv("data/log.csv")
    new_samp = json.loads(message.payload.decode())
    new_row = {'time':str_date_time, 'batteryLevel':new_samp["batteryLevel"], 'Speed':new_samp["Speed"], 'Servo':new_samp["Servo"]}
    df = df.append(new_row, ignore_index=True)
    df.to_csv("data/log.csv", index=False)
    # emit a mqtt_message event to the socket containing the message data
    socketio.emit('mqtt_message', data=data)
    print(data)


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/battery')
def battery():
    df = pd.read_csv("data/log.csv")
    fig = px.line(df[-10:], x="time", y="batteryLevel", title='Battery level')

    graphJSON = json.dumps(fig, cls=plotly.utils.PlotlyJSONEncoder)
    header="Battery level of the RC cart"
    description = """
    A academic study of the number of apples, oranges and bananas in the cities of
    San Francisco and Montreal would probably not come up with this chart.
    """
    return render_template('notdash2.html', graphJSON=graphJSON, header=header,description=description)

@app.route('/speed')
def speed():
    df = pd.read_csv("data/log.csv")
    fig = px.line(df[-10:], x="time", y="Speed", title='Motor Speed')

    graphJSON = json.dumps(fig, cls=plotly.utils.PlotlyJSONEncoder)
    header="Motor Speed"
    description = """
    The rumor that vegetarians are having a hard time in London and Madrid can probably not be
    explained by this chart.
    """
    return render_template('notdash2.html', graphJSON=graphJSON, header=header,description=description)

@app.route('/servo')
def servo():
    df = pd.read_csv("data/log.csv")
    fig = px.line(df[-10:], x="time", y="Servo", title='Servo angle')

    graphJSON = json.dumps(fig, cls=plotly.utils.PlotlyJSONEncoder)
    header="Servo angle"
    description = """
    The rumor that vegetarians are having a hard time in London and Madrid can probably not be
    explained by this chart.
    """
    return render_template('notdash2.html', graphJSON=graphJSON, header=header,description=description)
