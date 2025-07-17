# -*- coding: utf-8 -*-
"""
Created on Tue Jun 24 12:49:08 2025

@author: ASUS
"""

# -*- coding: utf-8 -*-
"""
Created on Mon Mar 10 14:14:28 2025

@author: ASUS
"""

import json
import os
import time
import requests
import logging
import paho.mqtt.client as mqtt
from threading import Thread, Lock

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# File path for config
CONFIG_FILE = "config.json"

# Global variables and lock
config_lock = Lock()
DEVICE_TOKENS = {}
BUTTON_TO_DEVICE = {}
TTN_APP_ID = ""
TTN_API_KEY = ""
TTN_BROKER = ""
TTN_PORT = ""
THINGSBOARD_SERVER = ""
POST_URL_TEMPLATE = ""
PUT_URL_TEMPLATE = ""

# Dictionary to track active alerts
active_alerts = {}

# Initial load of config
def load_static_config():
    global TTN_APP_ID, TTN_API_KEY, TTN_BROKER, TTN_PORT, THINGSBOARD_SERVER, POST_URL_TEMPLATE, PUT_URL_TEMPLATE
    try:
        with open(CONFIG_FILE, "r") as f:
            config = json.load(f)
        TTN_APP_ID = config["TTN_APP_ID"]
        TTN_API_KEY = config["TTN_API_KEY"]
        TTN_BROKER = config["TTN_BROKER"]
        TTN_PORT = config["TTN_PORT"]
        THINGSBOARD_SERVER = config["THINGSBOARD_SERVER"]
        POST_URL_TEMPLATE = config["POST_URL_TEMPLATE"]
        PUT_URL_TEMPLATE = config["PUT_URL_TEMPLATE"]
        logging.info("📦 Static configuration loaded.")
    except Exception as e:
        logging.error(f"❌ Failed to load static config: {e}")
        raise e

# Reload only the device mappings
def reload_device_mappings():
    global DEVICE_TOKENS, BUTTON_TO_DEVICE
    try:
        with open(CONFIG_FILE, "r") as f:
            config = json.load(f)
        with config_lock:
            DEVICE_TOKENS = config.get("DEVICE_TOKENS", {})
            BUTTON_TO_DEVICE = config.get("BUTTON_TO_DEVICE", {})
        logging.info("🔁 Device mappings reloaded.")
    except Exception as e:
        logging.error(f"❌ Failed to reload device mappings: {e}")

# Background thread to reload device mappings
def config_watcher():
    while True:
        reload_device_mappings()
        time.sleep(10)

# Send telemetry to ThingsBoard
def send_data_to_thingsboard(device_token, data):
    url = f"{THINGSBOARD_SERVER}/api/v1/{device_token}/telemetry"
    headers = {"Content-Type": "application/json"}
    try:
        response = requests.post(url, headers=headers, json=data)
        if response.status_code == 200:
            logging.info(f"✅ Data sent to ThingsBoard for token {device_token}")
        else:
            logging.error(f"❌ Failed to send telemetry: {response.status_code}, {response.text}")
    except Exception as e:
        logging.error(f"❌ Error sending to ThingsBoard: {e}")

# Monitor active alerts and close them after 2 minutes
def monitor_alerts():
    while True:
        current_time = time.time()
        to_remove = []

        for device_name, alert_info in active_alerts.items():
            elapsed = current_time - alert_info["timestamp"]
            if elapsed >= 120:
                incident_id = alert_info["incident_id"]
                put_url = PUT_URL_TEMPLATE.format(incident_id)
                logging.info(f"🔄 Closing incident {incident_id} for {device_name}")
                try:
                    response = requests.put(put_url, json={})
                    if response.status_code == 200:
                        logging.info(f"✅ Incident {incident_id} closed for {device_name}")
                        to_remove.append(device_name)
                    else:
                        logging.error(f"❌ Failed to close incident {incident_id}: {response.status_code}, {response.text}")
                except Exception as e:
                    logging.error(f"❌ Error closing incident: {e}")

        for device_name in to_remove:
            del active_alerts[device_name]

        time.sleep(5)

# MQTT message handler
def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode("utf-8"))
        logging.info(f"📡 Received from TTN: {json.dumps(data, indent=2)}")

        device_id = data.get("end_device_ids", {}).get("device_id", "unknown_device")
        payload = data.get("uplink_message", {}).get("decoded_payload", {})
        rssi = data.get("uplink_message", {}).get("rx_metadata", [{}])[0].get("rssi")
        snr = data.get("uplink_message", {}).get("rx_metadata", [{}])[0].get("snr")
        consumed_airtime = data.get("uplink_message", {}).get("consumed_airtime")
        battery = payload.get("battery_percentage")
        message = payload.get("button_id")

        with config_lock:
            device_token = DEVICE_TOKENS.get(message)
            device_name = BUTTON_TO_DEVICE.get(message)

        # Send telemetry
        if device_token:
            telemetry_data = {
                "device_id": device_id,
                "rssi": rssi,
                "snr": snr,
                "consumed_airtime": consumed_airtime,
                "battery": battery
            }
            send_data_to_thingsboard(device_token, telemetry_data)

        # Trigger incident
        if device_name:
            logging.info(f"🔘 Button {message} pressed by {device_id} — triggering alert for {device_name}")
            post_url = POST_URL_TEMPLATE.format(device_name)
            try:
                response = requests.post(post_url, json={})
                if response.status_code == 200:
                    resp_json = response.json()
                    incident_id = resp_json.get("idIncident")
                    if incident_id is not None:
                        active_alerts[device_name] = {
                            "incident_id": incident_id,
                            "timestamp": time.time()
                        }
                        logging.info(f"🚨 Incident {incident_id} started for {device_name}")
                    else:
                        logging.error(f"❌ No 'idIncident' in response: {resp_json}")
                else:
                    logging.error(f"❌ Failed to create incident: {response.status_code}, {response.text}")
            except Exception as e:
                logging.error(f"❌ Error sending alert request: {e}")
    except Exception as e:
        logging.error(f"❌ Failed to process message: {e}")

# MQTT connect callback
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        topic = f"v3/{TTN_APP_ID}/devices/+/up"
        logging.info(f"🔌 Connected to TTN MQTT broker. Subscribing to {topic}")
        client.subscribe(topic)
    else:
        logging.error(f"❌ MQTT connection failed with code {rc}")

# ======== MAIN PROGRAM ========
if __name__ == "__main__":
    try:
        load_static_config()
        reload_device_mappings()
    except Exception:
        logging.critical("🚫 Exiting due to config load failure.")
        exit(1)

    # Start background threads
    Thread(target=monitor_alerts, daemon=True).start()
    Thread(target=config_watcher, daemon=True).start()

    # MQTT setup
    client = mqtt.Client()
    client.username_pw_set(TTN_APP_ID, TTN_API_KEY)
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        logging.info(f"🔄 Connecting to TTN MQTT at {TTN_BROKER}:{TTN_PORT}")
        client.connect(TTN_BROKER, TTN_PORT, 60)
        client.loop_forever()
    except Exception as e:
        logging.error(f"❌ MQTT connection error: {e}")