from re import S
from matplotlib.pyplot import subplot
from pynmeagps import NMEAReader as nmr
import os
import numpy as np
import service_identity


# msg = NMEAReader.parse("$GPGSV,4,1,14,01,42,170,29,03,00,176,,04,21,196,18,07,56,321,15*70")
# print(msg)
# print(msg.az_01)
data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/hk_data" 
data_file = data_dir + "/20210517.light-urban.tste.ublox.m8t.GC.nmea"

def errhandler(err):
    """
    Handles errors output by iterator.
    """

    print(f"\nERROR: {err}\n")


def read_sv(stream):
    """
    Reads and parses UBX message data from stream.
    """
    # pylint: disable=unused-variable

    msg_ct = 0
    failed_ct = 0
    id_set = set()
    time_list = []

    sv_num_msgs = 0
    sv_set = {}
    car_data = {"lat": [], "lon": [], "time":[]}

    for fline in stream:
        msg_ct += 1
        try:
            msg = nmr.parse(fline)
            msg_id = msg.msgID
            talker = msg.talker
            id_set.add(msg.identity)
            if msg_id == "GGA":
                time = msg.time # python datetime.time
                time = time.hour*3600 + time.minute*60 + time.second
                time_list.append(time) # convert to seconds

                if not (msg.lon == '' and msg.lat== ''):
                    car_data["lon"].append(msg.lon)
                    car_data["lat"].append(msg.lat)
                car_data["time"].append(time)

            if msg_id == "GSV":
                sv_num_msgs = msg.numMsg
                msg_num = msg.msgNum
                num_sv = msg.numSV

                payload = msg.payload
                sv_payload = payload[3:]
                for i in range(0, len(sv_payload), 4):
                    sv_id = talker + sv_payload[i]
                    elev = sv_payload[i+1]
                    azim = sv_payload[i+2]
                    cno = sv_payload[i+3]
                    if not (elev == '' or azim == ''):
                        if sv_id in sv_set:
                            sv_set[sv_id]["elev"].append(elev)
                            sv_set[sv_id]["azim"].append(azim)
                            sv_set[sv_id]["cno"].append(cno)
                            sv_set[sv_id]["time"].append(time_list[-1])
                        else:
                            sv_set[sv_id] = {
                                "elev": [elev],
                                "azim": [azim],
                                "cno": [cno],
                                "time": [time_list[-1]],
                            }
        except:
            failed_ct += 1
    print(f"\n{msg_ct} messages read.\n{msg_ct - failed_ct} messages parsed")

    car_data["lat"] = np.array(car_data["lat"]).astype(float)
    car_data["lon"] = np.array(car_data["lon"]).astype(float)

    for key in sv_set:
        sv_set[key]["elev"] = np.array(sv_set[key]["elev"]).astype(float)
        sv_set[key]["azim"] = np.array(sv_set[key]["azim"]).astype(float)
        sv_set[key]["mean"] = (np.mean(sv_set[key]["elev"]), np.mean(sv_set[key]["azim"]))
        # sv_set[key]["cno"] = np.array(sv_set[key]["cno"]).astype(float)
        print(key, sv_set[key]["mean"])

    return sv_set, car_data

def test():
    msg = nmr.parse("$GPGSV,4,1,14,01,42,170,29,03,00,176,,04,21,196,18,07,56,321,15*70")

    payload = msg.payload

    sv = payload[3:]
    print(len(sv))
    print(sv)

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    # filename = "/home/zpyang/git/gps_sim/src/multipath_sim/data/hk_data/20210517.light-urban.tste.ublox.m8t.GC.nmea"
    filename = "/home/zpyang/git/gps_sim/src/multipath_sim/data/hk_data/20210517.light-urban.tste.ublox.f9p.nmea"
    # filename = "/home/zpyang/git/gps_sim/src/multipath_sim/data/hk_data/20210517.light-urban.tste.ublox.m8t.GR.nmea"
    # filename = "/home/zpyang/git/gps_sim/src/multipath_sim/data/hk_data/20210517.light-urban.tste.ublox.m8t.GEJ.nmea"
    print(f"\nOpening file {filename}...\n")
    with open(filename, "rb") as fstream:
        sv_set, car_data = read_sv(fstream)
        print(sv_set.keys())
        
        # sv_id = "GP16"
        # plt.figure()
        # for sv_id in sv_set.keys():
        #     plt.plot(sv_set[sv_id]["time"], np.array(sv_set[sv_id]["elev"]), label="elev")
        #     plt.ylim((0,90))
        #     plt.title("elevation")
        # plt.figure()
        # for sv_id in sv_set.keys():
        #     plt.plot(sv_set[sv_id]["time"], np.array(sv_set[sv_id]["azim"]), label="azim")
        #     plt.ylim((0,360))
        #     plt.title("azimuth")
        # # plt.legend()

        fig, ax = plt.subplots(subplot_kw={"projection": "polar"})
        for sv_id in sv_set.keys():
            ax.plot(np.deg2rad(sv_set[sv_id]["azim"]), sv_set[sv_id]["elev"])
            ax.set_rlim(bottom=90, top=0)
            ax.grid(True)
            ax.set_title("Skyplot")

        plt.figure()
        plt.plot(car_data["lon"], car_data["lat"], '.')
        plt.xlabel("lon")
        plt.ylabel("lat")

        plt.show()

    print("\nProcessing Complete")