"""Cliente paho compatible con 1.x y 2.x (mismo enfoque que el ejemplo v2)."""


def make_client(client_id='', will=None):
    """will = (topic, payload, qos, retain) opcional."""
    import paho.mqtt.client as mqtt

    if hasattr(mqtt, 'CallbackAPIVersion'):
        client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
            client_id=client_id, clean_session=True)
    else:
        client = mqtt.Client(client_id=client_id, clean_session=True)
    client.reconnect_delay_set(min_delay=1, max_delay=3)
    if will is not None:
        topic, payload, qos, retain = will
        client.will_set(topic, payload, qos=qos, retain=retain)
    return client
