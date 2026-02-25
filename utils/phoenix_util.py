from phoenix6 import status_code

def apply_config(device, config, i=10, t=0.2):
    for _ in range(i):
        res = device.configurator.apply(config, t)
    if res == status_code.StatusCode.OK:
        return