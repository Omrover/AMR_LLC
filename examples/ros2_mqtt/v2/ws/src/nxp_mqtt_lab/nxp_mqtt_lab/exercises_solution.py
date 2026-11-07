from .protocol import angular_speed, duty as checked_duty


def wheel_speed(previous, current, ticks_per_rev):
    return angular_speed(previous, current, ticks_per_rev)


def pwm_pair(duty):
    value = checked_duty(duty)
    return [value, value]
