import requests


def send_print(filename):
    """sends file to printer to print returns true if successful"""
    try:
        response = requests.post(f"http://mainsailos.local/printer/print/start?filename={filename}")
    except:
        raise Exception("cant connect to printer")
    if "result" in response.json():
        return True
    else:
        return False


def get_print_status():
    try:
        response = requests.get("http://mainsailos.local/printer/objects/query?print_stats=state")
    except Exception as e:
        return f"PRINTER LIKELY DISCONNECTED SEE EXCEPTION RAISED:\n({e})"
    else:
        text = response.json()
        status = text["result"]["status"]["print_stats"]["state"]
        return status
def send_gcode(gcode):
    """send gcode to printer,lines of code must be seperated with \n"""
    response = requests.post(f"http://mainsailos.local/printer/gcode/script?script={gcode}")
    print(response.json())
