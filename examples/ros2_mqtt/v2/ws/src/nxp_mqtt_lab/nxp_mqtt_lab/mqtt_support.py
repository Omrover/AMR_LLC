import paho.mqtt.client as mqtt


def make_client():
    # Paho 1.x/2.x compatibility; callback signatures use the v1 API.
    if hasattr(mqtt, 'CallbackAPIVersion'):
        client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
            clean_session=True,
        )
    else:
        client = mqtt.Client(clean_session=True)
    client.reconnect_delay_set(min_delay=1, max_delay=3)
    client.max_queued_messages_set(1)
    return client
