"""Edit this file, then restart the monitor (symlink-install needs no rebuild)."""


def wheel_speed(previous, current, ticks_per_rev):
    """Return rad/s, or None for first sample, reboot or nonpositive dt."""
    ## TODO:: add code here  # noqa: E266
    # Samples have ticks, uptime_ms and boot_id. Use MCU time, not MQTT arrival time.
    raise NotImplementedError('Complete wheel_speed or select use_solution:=true')


def pwm_pair(duty):
    """Return [left, right], rejecting nonfinite or |duty| > 0.2 (optional)."""
    ## TODO:: add code here  # noqa: E266
    raise NotImplementedError('Complete the optional PWM exercise')
